use nalgebra_glm::{Vec3, normalize, cross, dot};
use minifb::{Key, Window, WindowOptions};
use std::time::Duration;
use std::f32::consts::PI;

mod framebuffer;
mod ray_intersect;
mod color;
mod camera;
mod light;
mod material;
mod cube;

use framebuffer::Framebuffer;
use ray_intersect::Intersect;
use ray_intersect::RayIntersect;
use color::Color;
use camera::Camera;
use light::Light;
use material::Material;
use cube::Cube;

fn clamp01(x: f32) -> f32 { if x < 0.0 { 0.0 } else if x > 1.0 { 1.0 } else { x } }

fn local_shading(hit: &Intersect, view_dir: &Vec3, lights: &[Light]) -> Color {
    let mut accum = Color::new(0, 0, 0);
    for light in lights {
        let l = (light.position - hit.point).normalize();
        let n_dot_l = clamp01(dot(&hit.normal, &l));

        let diff = hit.material.albedo[0] * n_dot_l * light.intensity;
        let d = hit.material.diffuse * diff;

        let h = normalize(&(l + *view_dir));
        let n_dot_h = clamp01(dot(&hit.normal, &h));
        let spec = hit.material.albedo[1] * n_dot_h.powf(hit.material.specular);
        let s = Color::new(255, 255, 255) * spec;

        accum = accum + d + s;
    }
    accum
}


fn saturate(x: f32) -> f32 { x.max(0.0).min(1.0) }
fn mix_color(a: Color, b: Color, t: f32) -> Color { a * (1.0 - t) + b * t }


fn sun_dir() -> Vec3 {
    normalize(&Vec3::new(-0.25, 0.35,  1.0)) //Para ver el arcoiris
    // normalize(&Vec3::new(-0.25, 0.35, -1.0)) // Para ver el sol
}

fn spectrum_rgb(t_in: f32) -> Color {
    let t = saturate(t_in);
    let stops = [
        Color::from_hex(0x6a00ff), // violeta
        Color::from_hex(0x4b00ff), // índigo
        Color::from_hex(0x0066ff), // azul
        Color::from_hex(0x00cc66), // verde
        Color::from_hex(0xffdd00), // amarillo
        Color::from_hex(0xff8800), // naranja
        Color::from_hex(0xff0033), // rojo
    ];
    let n = (stops.len() - 1) as f32;
    let x = t * n;
    let i = x.floor() as usize;
    let f = x - i as f32;
    if i >= stops.len() - 1 { stops[stops.len() - 1] } else { mix_color(stops[i], stops[i + 1], f) }
}


fn sky_color(dir: &Vec3) -> Color {
    let ty = saturate((dir.y + 1.0) * 0.5);
    let zenit = Color::from_hex(0x7fb6ff);
    let mid   = Color::from_hex(0xaed1ff);
    let hori  = Color::from_hex(0xd9e7ff);
    let base = if ty > 0.6 {
        let k = (ty - 0.6) / 0.4;
        mix_color(zenit, mid, k)
    } else {
        let k = ty / 0.6;
        mix_color(mid, hori, k)
    };


    let sdir   = sun_dir();
    let cosang = dot(dir, &sdir).clamp(-1.0, 1.0);
    let core   = saturate(cosang);
    let sun_core = core.powf(32.0);     // tamaño del disco (baja el exponente → más grande)
    let sun_halo = core.powf(8.0) * 0.45;
    let sun_col  = Color::from_hex(0xfff2c6);
    let sun      = sun_col * (2.2 * sun_core + 0.9 * sun_halo);

    let anti   = -sdir;
    let ct     = dot(dir, &anti).clamp(-1.0, 1.0);
    let theta  = ct.acos(); // radianes


    let center1 = 42.0_f32.to_radians();
    let sigma1  = 1.8_f32.to_radians();
    let w1 = ( -((theta - center1).powi(2)) / (2.0 * sigma1 * sigma1) ).exp();
    // mapear 40°..44° → 0..1
    let phase1 = saturate((theta - (center1 - 2.0_f32.to_radians())) / (4.0_f32.to_radians()));
    let col1   = spectrum_rgb(phase1);
    let rainbow1 = col1 * (0.60 * w1);


    let center2 = 51.0_f32.to_radians();
    let sigma2  = 2.2_f32.to_radians();
    let w2 = ( -((theta - center2).powi(2)) / (2.0 * sigma2 * sigma2) ).exp();
    let phase2 = 1.0 - saturate((theta - (center2 - 2.0_f32.to_radians())) / (4.0_f32.to_radians()));
    let col2   = spectrum_rgb(phase2);
    let rainbow2 = col2 * (0.35 * w2);

    base + sun + rainbow1 + rainbow2
}



fn primary_ray_dir(
    x: usize, y: usize, w: usize, h: usize, fov: f32,
    cam_eye: &Vec3, cam_center: &Vec3, cam_up: &Vec3
) -> Vec3 {
    let forward = normalize(&(cam_center - cam_eye));
    let right = normalize(&cross(&forward, cam_up));
    let up = cross(&right, &forward);

    let aspect = w as f32 / h as f32;
    let px = (2.0 * ((x as f32 + 0.5) / w as f32) - 1.0) * (fov * 0.5).tan() * aspect;
    let py = (1.0 - 2.0 * ((y as f32 + 0.5) / h as f32)) * (fov * 0.5).tan();

    normalize(&(forward + right * px + up * py))
}

fn nearest_hit_cubes(ray_o: &Vec3, ray_d: &Vec3, cubes: &[Cube]) -> Option<Intersect> {
    let mut min_t = f32::INFINITY;
    let mut best: Option<Intersect> = None;
    for c in cubes {
        let isect = c.ray_intersect(ray_o, ray_d);
        if isect.is_intersecting && isect.distance < min_t && isect.distance > 1e-3 {
            min_t = isect.distance;
            best = Some(isect);
        }
    }
    best
}

fn trace_with_reflection_refraction_cubes(
    ray_o: Vec3, ray_d: Vec3, cubes: &[Cube], lights: &[Light], depth: u32
) -> Color {
    if depth == 0 { return Color::new(0, 0, 0); }

    if let Some(hit) = nearest_hit_cubes(&ray_o, &ray_d, cubes) {
        let view_dir = (-ray_d).normalize();

        let mut color_local = local_shading(&hit, &view_dir, lights);

        let mut color_reflect = Color::new(0, 0, 0);
        if hit.material.reflectivity > 0.0 {
            let r_dir = Material::reflect_dir(&ray_d, &hit.normal).normalize();
            let r_orig = hit.point + r_dir * 1e-3;
            color_reflect =
                trace_with_reflection_refraction_cubes(r_orig, r_dir, cubes, lights, depth - 1);
        }

        if hit.material.transparency > 0.0 {
            let eta_i = 1.0;
            let eta_t = hit.material.ior;
            let cos_i = clamp01(-dot(&ray_d, &hit.normal));
            let fresnel = Material::fresnel_schlick(cos_i, eta_i, eta_t);

            if let Some(t_dir) = Material::refract_dir(&ray_d, &hit.normal, eta_i, eta_t) {
                let t_orig = hit.point + t_dir * 1e-3;
                let color_refract =
                    trace_with_reflection_refraction_cubes(t_orig, t_dir, cubes, lights, depth - 1);

                let base_w = (1.0 - hit.material.reflectivity - hit.material.transparency).max(0.0);
                let refl_w = fresnel.max(hit.material.reflectivity);
                let refr_w = (1.0 - fresnel) * hit.material.transparency;

                return color_local * base_w + color_reflect * refl_w + color_refract * refr_w;
            } else {
                let base_w = (1.0 - hit.material.reflectivity).max(0.0);
                return color_local * base_w + color_reflect * hit.material.reflectivity;
            }
        }

        if hit.material.reflectivity > 0.0 {
            let base_w = (1.0 - hit.material.reflectivity).max(0.0);
            return color_local * base_w + color_reflect * hit.material.reflectivity;
        }

        return color_local;
    }

    sky_color(&ray_d)
}


fn build_min_city_scene() -> (Vec<Cube>, Vec<Light>) {
    let mut cubes: Vec<Cube> = Vec::new();
    let mut lights: Vec<Light> = Vec::new();

    let mut asphalt = Material::new(Color::from_hex(0x121316), 96.0, [0.8, 0.2]);
    asphalt.reflectivity = 0.55;
    asphalt.transparency = 0.0;
    asphalt.ior = 1.0;

    let mut concrete = Material::new(Color::from_hex(0x3a3f45), 32.0, [0.9, 0.1]);
    concrete.reflectivity = 0.05;
    concrete.transparency = 0.0;
    concrete.ior = 1.0;

    let mut glass = Material::new(Color::from_hex(0x98c9ff), 64.0, [0.0, 1.0]);
    glass.reflectivity = 0.06;
    glass.transparency = 0.9;
    glass.ior = 1.52;

    cubes.push(Cube {
        min: Vec3::new(-500.0, -0.1, -500.0),
        max: Vec3::new( 500.0,  0.0,  500.0),
        material: asphalt,
    });

    let street_half_x = 8.0;   
    let building_w    = 6.0;   
    let building_d    = 8.0;   
    let start_z       = -12.0;
    let step_z        = -12.0; 
    let blocks        = 4;     

 
    let neon = [
        Color::from_hex(0x00ffff),
        Color::from_hex(0xff00ff),
        Color::from_hex(0x66ccff),
        Color::from_hex(0xff66cc),
        Color::from_hex(0x00ffea),
    ];


    let mut seed: u32 = 1234567;
    let mut rnd01 = || {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        (seed as f32 / u32::MAX as f32).abs()
    };


    for side in [-1.0_f32, 1.0_f32] {
        let cx_base = side * (street_half_x + building_w * 0.5);

        for i in 0..blocks {
            let zc   = start_z + step_z * i as f32;
            let h    = 8.0 + rnd01() * 6.0; 
            let ytop = h;


            let min = Vec3::new(cx_base - building_w * 0.5, 0.0, zc - building_d * 0.5);
            let max = Vec3::new(cx_base + building_w * 0.5, ytop, zc + building_d * 0.5);
            cubes.push(Cube { min, max, material: concrete });


            let inner_face_x = if side < 0.0 { max.x } else { min.x };
            let (wx0, wx1) = if side < 0.0 {
                (inner_face_x + 0.02, inner_face_x + 0.06)
            } else {
                (inner_face_x - 0.06, inner_face_x - 0.02)
            };

            let cols = [-1.8_f32, 0.0, 1.8];
            let rows = [2.5_f32, 5.0];
            for &cz in &cols {
                for &cy in &rows {
                    let wz = zc + cz;
                    let wy = cy.min(ytop - 0.7);
                    cubes.push(Cube {
                        min: Vec3::new(wx0, wy - 0.45, wz - 0.65),
                        max: Vec3::new(wx1, wy + 0.45, wz + 0.65),
                        material: glass,
                    });
                }
            }


            let panel_y  = (3.0 + rnd01() * 3.0).min(ytop - 1.0);
            let panel_z  = zc + if rnd01() < 0.5 { -1.5 } else { 1.5 };
            let (px0, px1, lx) = if side < 0.0 {
                (inner_face_x + 0.03, inner_face_x + 0.12, inner_face_x + 0.35)
            } else {
                (inner_face_x - 0.12, inner_face_x - 0.03, inner_face_x - 0.35)
            };

            cubes.push(Cube {
                min: Vec3::new(px0, panel_y - 0.9, panel_z - 1.2),
                max: Vec3::new(px1, panel_y + 0.9, panel_z + 1.2),
                material: glass,
            });

            let cidx = (i + if side < 0.0 {0} else {3}) % neon.len();
            let inten = 2.2 + rnd01() * 1.0;
            lights.push(Light::new(Vec3::new(lx, panel_y + 0.1, panel_z), neon[cidx], inten));


            if rnd01() > 0.4 {
                let top_y = (ytop - 0.5).max(3.5);
                let lz    = zc + if rnd01() < 0.5 { -0.8 } else { 0.8 };
                let lx2   = if side < 0.0 { inner_face_x + 0.6 } else { inner_face_x - 0.6 };
                let cidx2 = (i * 2 + 1) % neon.len();
                lights.push(Light::new(Vec3::new(lx2, top_y, lz), neon[cidx2], 1.8));
            }
        }
    }

    (cubes, lights)
}



fn draw_rain_overlay(fb: &mut Framebuffer, frame: u32) {
    let w = fb.width as isize;
    let h = fb.height as isize;

    let mut seed = frame.wrapping_mul(1664525).wrapping_add(1013904223);

    let streaks = ((w as u32) / 14).max(26) as usize;
    for _ in 0..streaks {
        // RNG
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let x = (seed % (w as u32)) as isize;

        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let len = 12 + (seed % 28) as isize;

        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let speed = 7 + (seed % 9) as isize;

        let y0 = ((frame as isize * speed) % (h + len)) - len;


        let c = match seed & 3 {
            0 => 0xE0E6EA,
            1 => 0xC9D6DF,
            _ => 0xB5C7D9,
        };


        let dx = if (seed & 7) < 3 { 1 } else { 0 };

        let mut xi = x;
        let mut yi = y0;
        for _ in 0..len {
            if xi >= 0 && yi >= 0 && xi < w && yi < h {
                fb.set_current_color(c);
                fb.point(xi as usize, yi as usize);
            }
            xi += dx;
            yi += 1;
        }
    }
}



fn main() {
    let width: usize = 960;
    let height: usize = 540;

    let mut fb = Framebuffer::new(width, height);
    let mut window = Window::new(
        "RayTracing",
        width, height, WindowOptions::default(),
    ).unwrap();


    let mut camera = Camera::new(
        Vec3::new(0.0, 3.0, 10.0), 
        Vec3::new(0.0, 4.0, -20.0),  
        Vec3::new(0.0, 1.0, 0.0),  
    );


    let (cubes, lights) = build_min_city_scene();

    let fov: f32 = PI / 3.0;
    let frame_delay = Duration::from_millis(0);
    let mut frame_count: u32 = 0;

    while window.is_open() {

        if window.is_key_down(Key::W) || window.is_key_down(Key::Equal) { camera.dolly(-0.15); }
        if window.is_key_down(Key::S) || window.is_key_down(Key::Minus) { camera.dolly(0.15); }


        for y in 0..height {
            for x in 0..width {
                let dir = primary_ray_dir(x, y, width, height, fov, &camera.eye, &camera.center, &camera.up);
                let color = trace_with_reflection_refraction_cubes(camera.eye, dir, &cubes, &lights, 3); // depth=3
                fb.set_current_color(color.to_hex());
                fb.point(x, y);
            }
        }

        draw_rain_overlay(&mut fb, frame_count);
        frame_count = frame_count.wrapping_add(1);

        window.update_with_buffer(&fb.buffer, width, height).unwrap();
        std::thread::sleep(frame_delay);
    }
}
