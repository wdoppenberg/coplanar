use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::window::WindowResolution;
use body_renderer::dem::DEMManager;
use body_renderer::MoonPlugin;

fn main() {
    let dem_manager =
        DEMManager::new("/Users/wdoppenberg/Downloads/Lunar_LRO_LOLA_Global_LDEM_118m_Mar2014.tif")
            .expect("Failed to initialize DEM manager");

    let default_plugins = DefaultPlugins
        .set(WindowPlugin {
            primary_window: Some(Window {
                title: "Moon Viewer".into(),
                resolution: WindowResolution::default(),
                ..default()
            }),
            ..default()
        })
        .set(LogPlugin {
            filter: "body_renderer=debug".into(),
            ..Default::default()
        });
    App::new()
        .add_plugins(default_plugins)
        .insert_resource(dem_manager)
        .add_plugins(MoonPlugin)
        .run();
}
