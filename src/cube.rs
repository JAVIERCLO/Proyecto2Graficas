use nalgebra_glm::Vec3;
use crate::ray_intersect::{RayIntersect, Intersect};
use crate::material::Material;

pub struct Cube {
    pub min: Vec3,
    pub max: Vec3,
    pub material: Material,
}

impl Cube {
    /// Crea un cubo por centro y tamaño de arista.
    pub fn from_center_size(center: Vec3, size: f32, material: Material) -> Self {
        let he = Vec3::new(size * 0.5, size * 0.5, size * 0.5);
        Self { min: center - he, max: center + he, material }
    }
}

impl RayIntersect for Cube {
    fn ray_intersect(&self, ray_origin: &Vec3, ray_dir: &Vec3) -> Intersect {
        let inv = Vec3::new(
            if ray_dir.x != 0.0 { 1.0 / ray_dir.x } else { f32::INFINITY },
            if ray_dir.y != 0.0 { 1.0 / ray_dir.y } else { f32::INFINITY },
            if ray_dir.z != 0.0 { 1.0 / ray_dir.z } else { f32::INFINITY },
        );

        let t0 = (self.min - ray_origin).component_mul(&inv);
        let t1 = (self.max - ray_origin).component_mul(&inv);

        let tmin = t0.zip_map(&t1, |a, b| a.min(b));
        let tmax = t0.zip_map(&t1, |a, b| a.max(b));

        let t_near = tmin.x.max(tmin.y).max(tmin.z);
        let t_far  = tmax.x.min(tmax.y).min(tmax.z);

        if t_far < 0.0 || t_near > t_far {
            return Intersect::empty();
        }

        let t = if t_near > 1e-6 { t_near } else { t_far };
        if t <= 1e-6 {
            return Intersect::empty();
        }

        let point = ray_origin + ray_dir * t;

        // Normal por cara con tolerancia
        let eps = 1e-4;
        let normal = if (point.x - self.min.x).abs() < eps { Vec3::new(-1.0, 0.0, 0.0) }
        else if (point.x - self.max.x).abs() < eps { Vec3::new( 1.0, 0.0, 0.0) }
        else if (point.y - self.min.y).abs() < eps { Vec3::new( 0.0,-1.0, 0.0) }
        else if (point.y - self.max.y).abs() < eps { Vec3::new( 0.0, 1.0, 0.0) }
        else if (point.z - self.min.z).abs() < eps { Vec3::new( 0.0, 0.0,-1.0) }
        else { Vec3::new( 0.0, 0.0, 1.0) };

        Intersect::new(point, normal, t, self.material)
    }
}
