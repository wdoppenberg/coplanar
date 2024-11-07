use bevy::prelude::Resource;
use bevy::prelude::*;
use flume::{unbounded, Receiver, Sender};
use gdal::Dataset;
use nalgebra::{Point3, Vector3};
use std::collections::HashMap;
use std::error::Error;
use std::path::Path;
use std::thread;

/// Represents a single elevation point in the DEM
#[derive(Debug, Clone, Copy)]
pub struct ElevationPoint {
    pub position: Point3<f32>,
    pub normal: Vector3<f32>,
}

/// Represents a single tile in the DEM pyramid
#[derive(Debug, Clone)]
pub struct DEMTile {
    pub width: usize,
    pub height: usize,
    pub elevation_data: Vec<f32>,
    pub min_elevation: f32,
    pub max_elevation: f32,
}

type TileMap = HashMap<TileKey, TileData>;

#[derive(Resource)]
pub struct DEMManager {
    tile_request_sender: Sender<TileRequest>,
    tile_map: TileMap,
    tile_size: usize,
    width: usize,
    height: usize,
}

#[derive(Copy, Debug, Clone, Eq, PartialEq, Hash)]
pub struct TileKey {
    x: usize,
    y: usize,
}

struct TileRequest {
    pub key: TileKey,
    offset_x: usize,
    offset_y: usize,
    size_x: usize,
    size_y: usize,
    tx: oneshot::Sender<(TileKey, TileData)>,
}

#[derive(Debug)]
pub struct TileData {
    pub elevation_data: Vec<f32>,
    pub width: usize,
    pub height: usize,
    pub last_used: std::time::Instant,
}

impl DEMManager {
    pub fn new<P: AsRef<Path> + Clone + Send + 'static>(path: P) -> Result<Self, Box<dyn Error>> {
        // Open dataset temporarily to get dimensions
        let dataset = Dataset::open(&path)?;
        let band = dataset.rasterband(1)?;
        let (width, height) = dataset.raster_size();

        info!("DEM Dataset has {} data type.", band.band_type());

        // Drop the dataset before spawning the thread
        drop(dataset);

        let (req_tx, req_rx) = unbounded();

        let worker_path = path.clone();
        let tile_map = HashMap::new();
        thread::spawn(move || {
            gdal_worker_thread(worker_path, req_rx.clone());
        });

        Ok(Self {
            tile_request_sender: req_tx,
            tile_map,
            tile_size: 1024,
            width,
            height,
        })
    }

    pub fn get_elevation(&mut self, lat: f32, lon: f32) -> f32 {
        let (x, y) = self.latlon_to_pixel(lat, lon);

        // Check if coordinates are within bounds
        if x >= self.width || y >= self.height {
            return 0.0;
        }

        let tile_key = self.get_tile_key(x, y);
        let tile_size = self.tile_size;

        if !self.tile_map.contains_key(&tile_key) {
            let offset_x = tile_key.x * tile_size;
            let offset_y = tile_key.y * tile_size;

            // Calculate actual tile size (handle boundaries)
            let size_x = tile_size.min(self.width - offset_x);
            let size_y = tile_size.min(self.height - offset_y);

            let (tx, rx) = oneshot::channel();

            let send_res = self.tile_request_sender.send(TileRequest {
                key: tile_key.clone(),
                offset_x,
                offset_y,
                size_x,
                size_y,
                tx,
            });

            if let Err(e) = send_res {
                error!("Something went wrong when sending tile data request: {}", e)
            }

            let (tile_key, tile_data) = rx.recv().expect("Could not receive tile data.");

            self.tile_map.insert(tile_key, tile_data);
        }

        if let Some(tile) = self.tile_map.get(&tile_key) {
            let local_x = x - (tile_key.x * tile_size);
            let local_y = y - (tile_key.y * tile_size);

            if local_x < tile.width && local_y < tile.height {
                let elev = tile.elevation_data[local_y * tile.width + local_x];
                trace!("Found elevation for lat {lat} lon {lon} -> ({x}, {y}): {elev}");
                return elev;
            }
        }

        0.0 // Default elevation if tile not yet loaded or coordinates invalid
    }

    fn latlon_to_pixel(&self, lat: f32, lon: f32) -> (usize, usize) {
        // Clamp latitude to [-90, 90] and longitude to [-180, 180]
        let lat = lat.clamp(-90.0, 90.0);
        let lon = lon.clamp(-180.0, 180.0);

        let x = ((lon + 180.0) / 360.0 * self.width as f32) as usize;
        let y = ((90.0 - lat) / 180.0 * self.height as f32) as usize;

        (x.min(self.width - 1), y.min(self.height - 1))
    }

    fn get_tile_key(&self, x: usize, y: usize) -> TileKey {
        TileKey {
            x: x / self.tile_size,
            y: y / self.tile_size,
        }
    }

    pub fn get_loaded_tile_count(&self) -> usize {
        self.tile_map.len()
    }

    pub fn is_tile_loaded(&self, key: &TileKey) -> bool {
        self.tile_map.contains_key(key)
    }
}

fn gdal_worker_thread<P: AsRef<Path>>(path: P, rx: Receiver<TileRequest>) {
    let dataset = match Dataset::open(&path) {
        Ok(ds) => ds,
        Err(_) => return,
    };

    let band = match dataset.rasterband(1) {
        Ok(b) => b,
        Err(_) => return,
    };

    while let Ok(request) = rx.recv() {
        // Allocate buffer of exact size needed
        let mut elevation_data = vec![0.0f32; request.size_x * request.size_y];

        if let Ok(()) = band.read_into_slice(
            (request.offset_x as isize, request.offset_y as isize),
            (request.size_x, request.size_y),
            (request.size_x, request.size_y),
            &mut elevation_data,
            None,
        ) {
            let tile_data = TileData {
                elevation_data,
                width: request.size_x,
                height: request.size_y,
                last_used: std::time::Instant::now(),
            };
            if let Err(_) = request.tx.send((request.key, tile_data)) {
                error!("Failed to send tile data");
            }
            // For now just print something to see it working
            trace!(
                "Loaded tile at ({}, {})",
                request.offset_x,
                request.offset_y
            );
        }
    }
}
