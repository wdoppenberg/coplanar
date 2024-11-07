use bevy::math::Vec3;

// Helper function to convert lat/lon to 3D point on sphere
pub fn latlon_to_sphere(lat: f32, lon: f32, radius: f32) -> Vec3 {
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();
    let cos_lat = lat_rad.cos();

    Vec3::new(
        radius * cos_lat * lon_rad.cos(),
        radius * lat_rad.sin(),
        radius * cos_lat * lon_rad.sin(),
    )
}

pub fn latlon_to_ellipsoid(lat: f32, lon: f32, semimajor: f32, semiminor: f32) -> Vec3 {
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();

    let denominator = (semimajor * semimajor * lat_rad.cos())
        * (semimajor * semimajor * lat_rad.cos())
        + (semiminor * semiminor * lat_rad.sin()) * (semiminor * semiminor * lat_rad.sin());
    let radius = semimajor * semimajor * semiminor / denominator.sqrt();

    Vec3::new(
        radius * lat_rad.cos() * lon_rad.cos(),
        radius * lat_rad.sin(),
        radius * lat_rad.cos() * lon_rad.sin(),
    )
}
