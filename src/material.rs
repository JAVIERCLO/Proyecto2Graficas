use nalgebra_glm::{Vec3, dot};
use crate::color::Color;

#[derive(Debug, Clone, Copy)]
pub struct Material {
  pub diffuse: Color,
  pub specular: f32,
  pub albedo: [f32; 2], 

  pub ior: f32,           
  pub reflectivity: f32,  
  pub transparency: f32, 
}

impl Material {
  pub fn new(
    diffuse: Color,
    specular: f32,
    albedo: [f32; 2],
  ) -> Self {
    Self {
      diffuse,
      specular,
      albedo,
      ior: 1.0,
      reflectivity: 0.0,
      transparency: 0.0,
    }
  }

  pub fn black() -> Self {
    Self::new(Color::from_hex(0x000000), 32.0, [1.0, 0.0])
  }

  pub fn water() -> Self {
    let mut m = Self::new(Color::from_hex(0x4aa3ff), 32.0, [0.1, 0.9]);
    m.ior = 1.333;
    m.transparency = 0.8;
    m.reflectivity = 0.02;
    m
  }


  pub fn fresnel_schlick(cos_i: f32, eta_i: f32, eta_t: f32) -> f32 {
    let r0 = ((eta_i - eta_t) / (eta_i + eta_t)).powi(2);
    r0 + (1.0 - r0) * (1.0 - cos_i).powi(5)
  }


  pub fn refract_dir(i: &Vec3, n: &Vec3, eta_i: f32, eta_t: f32) -> Option<Vec3> {
    let mut nn = *n;
    let mut etai = eta_i;
    let mut etat = eta_t;
    let mut cos_i = dot(i, n);

    if cos_i > 0.0 {
      nn = -*n;
      std::mem::swap(&mut etai, &mut etat);
      cos_i = -cos_i;
    }

    let eta = etai / etat;
    let c = -cos_i;
    let k = 1.0 - eta * eta * (1.0 - c * c);
    if k < 0.0 {
      None
    } else {
      Some(eta * *i + (eta * c - k.sqrt()) * nn)
    }
  }


  pub fn reflect_dir(i: &Vec3, n: &Vec3) -> Vec3 {
    *i - 2.0 * dot(i, n) * *n
  }



  pub fn asphalt_wet() -> Self {
    let mut m = Material::new(Color::from_hex(0x121316), 96.0, [0.8, 0.2]);
    m.reflectivity = 0.55;
    m.transparency = 0.0;
    m.ior = 1.0;
    m
  }


  pub fn concrete_matte() -> Self {
    let mut m = Material::new(Color::from_hex(0x3a3f45), 32.0, [0.9, 0.1]);
    m.reflectivity = 0.05;
    m.transparency = 0.0;
    m.ior = 1.0;
    m
  }


  pub fn glass_tinted() -> Self {
    let mut m = Material::new(Color::from_hex(0x98c9ff), 64.0, [0.0, 1.0]);
    m.reflectivity = 0.06;
    m.transparency = 0.9;
    m.ior = 1.52;
    m
  }

pub fn building_paint(hex: u32) -> Self {

    let mut paint_yellow = Material::building_paint(0xFFC83D); // amarillo vial
    paint_yellow.reflectivity = 0.05; // casi mate
    paint_yellow.specular = 32.0;

    let mut m = Material::new(Color::from_hex(hex), 48.0, [0.85, 0.15]);
    m.reflectivity = 0.12;  // un pelín de espejo para ver el sol
    m.transparency = 0.0;
    m.ior = 1.0;
    m

    
}

pub fn glass_day() -> Self {
    let mut m = Material::new(Color::from_hex(0xb8d9ff), 64.0, [0.0, 1.0]);
    m.reflectivity = 0.08;
    m.transparency = 0.88;
    m.ior = 1.52;
    m
}

pub fn metal_dark() -> Self {
    let mut m = Material::new(Color::from_hex(0x2a2f36), 96.0, [0.1, 0.9]);
    m.reflectivity = 0.35;
    m.transparency = 0.0;
    m.ior = 1.0;
    m
}


}
