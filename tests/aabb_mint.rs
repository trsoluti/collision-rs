extern crate mint;
extern crate collision;

use mint::{Point2, Point3};
use mint::{Vector2, Vector3};
use collision::{MintAabb2Trait as Aabb, MintAabb2 as Aabb2, MintAabb3 as Aabb3};
use collision::MintAabb3Trait;
use collision::{Contains, Continuous, Discrete, SurfaceArea, Union};
use collision::{MintLine2 as Line2, MintLine3 as Line3, Ray2, Ray3, Sphere};
use collision::{Plane, PlaneBound, Ray, Relation};
use collision::MintPoint2;

#[test]
fn test_general() {
    let aabb = Aabb2::new(
        [-20isize, 30isize],
        [10isize, -10isize],
    );
    assert_eq!(aabb.min(), [-20isize, -10isize].into());
    assert_eq!(aabb.max(), [10isize, 30isize].into());
    assert_eq!(aabb.dim(), [30isize, 40isize].into());
    assert_eq!(aabb.volume(), 30isize * 40isize);
    assert_eq!(aabb.center(), [-5isize, 10isize].into());

    assert!(aabb.contains(&[0isize, 0isize].into()));
    assert!(!aabb.contains(&[-50isize, -50isize].into()));
    assert!(!aabb.contains(&[50isize, 50isize].into()));

    assert_eq!(aabb.grow([0isize, 0isize].into()), aabb);
    assert_eq!(
        aabb.grow([100isize, 100isize].into()),
        Aabb2::new(
            [-20isize, -10isize],
            [100isize, 100isize],
        )
    );
    assert_eq!(
        aabb.grow([-100isize, -100isize].into()),
        Aabb2::new(
            [-100isize, -100isize],
            [10isize, 30isize],
        )
    );

    assert_eq!(
        aabb.add_margin([2isize, 2isize].into()),
        Aabb2::new(
            [-22isize, -12isize],
            [12isize, 32isize],
        )
    );

    let aabb = Aabb3::new(
        [-20isize, 30isize, 5isize],
        [10isize, -10isize, -5isize],
    );
    assert_eq!(aabb.min(), [-20isize, -10isize, -5isize].into());
    assert_eq!(aabb.max(), [10isize, 30isize, 5isize].into());
    assert_eq!(aabb.dim(), [30isize, 40isize, 10isize].into());
    assert_eq!(aabb.volume(), 30isize * 40isize * 10isize);
    assert_eq!(aabb.center(), [-5isize, 10isize, 0isize].into());

    assert!(aabb.contains(&[0isize, 0isize, 0isize].into()));
    assert!(!aabb.contains(&[-100isize, 0isize, 0isize].into()));
    assert!(!aabb.contains(&[100isize, 0isize, 0isize].into()));
    assert!(aabb.contains(&[9isize, 29isize, -1isize].into()));
    assert!(!aabb.contains(&[10isize, 30isize, 5isize].into()));
    assert!(aabb.contains(&[-20isize, -10isize, -5isize].into()));
    assert!(!aabb.contains(&[-21isize, -11isize, -6isize].into()));

    assert_eq!(
        aabb.add_v([1isize, 2isize, 3isize].into()),
        Aabb3::new(
            [-19isize, 32isize, 8isize],
            [11isize, -8isize, -2isize],
        )
    );

    assert_eq!(
        aabb.mul_s(2isize),
        Aabb3::new(
            [-40isize, -20isize, -10isize],
            [20isize, 60isize, 10isize],
        )
    );

    assert_eq!(
        aabb.mul_v([1isize, 2isize, 3isize].into()),
        Aabb3::new(
            [-20isize, -20isize, -15isize],
            [10isize, 60isize, 15isize],
        )
    );

    assert_eq!(
        aabb.add_margin([2isize, 2isize, 2isize].into()),
        Aabb3::new(
            [-22isize, -12isize, -7isize],
            [12isize, 32isize, 7isize],
        )
    );
}

#[test]
fn test_ray2_intersect() {
    let aabb = Aabb2::new([-5.0f32, 5.0], [5.0, 10.0]);
    let ray1 = Ray::new([0.0f32, 0.0].into(), Vector2::new(0.0, 1.0));
    let ray2 = Ray::new([-10.0f32, 0.0].into(), Vector2::new(2.5, 1.0));
    let ray3 = Ray::new([0.0f32, 0.0].into(), Vector2::new(-1.0, -1.0));
    let ray4 = Ray::new([3.0f32, 7.0].into(), Vector2::new(1.0, 1.0));

    assert_eq!(ray1.intersection(&aabb), Some([0.0, 5.0].into()));
    assert!(ray1.intersects(&aabb));
    assert_eq!(ray2.intersection(&aabb), Some([2.5, 5.0].into()));
    assert!(ray2.intersects(&aabb));
    assert_eq!(ray3.intersection(&aabb), None);
    assert!(!ray3.intersects(&aabb));
    assert_eq!(ray4.intersection(&aabb), Some([5.0, 9.0].into()));
    assert!(ray4.intersects(&aabb));
}

#[test]
fn test_parallel_ray3_should_not_intersect() {
    let aabb = Aabb3::<f32>::new([1.0, 1.0, 1.0].into(), [5.0, 5.0, 5.0].into());
    let ray_x = Ray::new([0.0f32, 0.0, 0.0].into(), [1.0, 0.0, 0.0].into());
    let ray_y = Ray::new([0.0f32, 0.0, 0.0].into(), [0.0, 1.0, 0.0].into());
    let ray_z = Ray::new([0.0f32, 0.0, 0.0].into(), [0.0, 0.0, 1.0].into());
    let ray_z_imprecise = Ray::new(
        [0.0f32, 0.0, 0.0].into(),
        [0.0001, 0.0001, 1.0].into(),
    );

    assert_eq!(ray_x.intersection(&aabb), None);
    assert!(!ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), None);
    assert!(!ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), None);
    assert!(!ray_z.intersects(&aabb));
    assert_eq!(ray_z_imprecise.intersection(&aabb), None);
    assert!(!ray_z_imprecise.intersects(&aabb));
}

#[test]
fn test_oblique_ray3_should_intersect() {
    let aabb = Aabb3::<f32>::new([1.0, 1.0, 1.0].into(), [5.0, 5.0, 5.0].into());
    let ray1 = Ray::new(
        [0.0f32, 0.0, 0.0].into(),
        [1.0, 1.0, 1.0].into().normalize(),
    );
    let ray2 = Ray::new([0.0f32, 6.0, 0.0].into(), [1.0, -1.0, 1.0].into());

    assert_eq!(ray1.intersection(&aabb), Some([1.0, 1.0, 1.0].into()));
    assert!(ray1.intersects(&aabb));
    assert_eq!(ray2.intersection(&aabb), Some([1.0, 5.0, 1.0].into()));
    assert!(ray2.intersects(&aabb));
}

#[test]
fn test_pointing_to_other_dir_ray3_should_not_intersect() {
    let aabb = Aabb3::<f32>::new([1.0, 1.0, 1.0].into(), [5.0, 5.0, 5.0].into());
    let ray_x = Ray::new(
        [0.0f32, 2.0, 2.0].into(),
        [-1.0, 0.01, 0.01].into(),
    );
    let ray_y = Ray::new(
        [2.0f32, 0.0, 2.0].into(),
        [0.01, -1.0, 0.01].into(),
    );
    let ray_z = Ray::new(
        [2.0f32, 2.0, 0.0].into(),
        [0.01, 0.01, -1.0].into(),
    );

    let ray_x2 = Ray::new([6.0f32, 2.0, 2.0].into(), [1.0, 0.0, 0.0].into());
    let ray_y2 = Ray::new([2.0f32, 6.0, 2.0].into(), [0.0, 1.0, 0.0].into());
    let ray_z2 = Ray::new([2.0f32, 2.0, 6.0].into(), [0.0, 0.0, 1.0].into());

    assert_eq!(ray_x.intersection(&aabb), None);
    assert!(!ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), None);
    assert!(!ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), None);
    assert!(!ray_z.intersects(&aabb));

    assert_eq!(ray_x2.intersection(&aabb), None);
    assert!(!ray_x2.intersects(&aabb));
    assert_eq!(ray_y2.intersection(&aabb), None);
    assert!(!ray_y2.intersects(&aabb));
    assert_eq!(ray_z2.intersection(&aabb), None);
    assert!(!ray_z2.intersects(&aabb));
}

#[test]
fn test_pointing_to_box_dir_ray3_should_intersect() {
    let aabb = Aabb3::<f32>::new([1.0, 1.0, 1.0].into(), [5.0, 5.0, 5.0].into());
    let ray_x = Ray::new([0.0f32, 2.0, 2.0].into(), [1.0, 0.0, 0.0].into());
    let ray_y = Ray::new([2.0f32, 0.0, 2.0].into(), [0.0, 1.0, 0.0].into());
    let ray_z = Ray::new([2.0f32, 2.0, 0.0].into(), [0.0, 0.0, 1.0].into());

    let ray_x2 = Ray::new([6.0f32, 2.0, 2.0].into(), [-1.0, 0.0, 0.0].into());
    let ray_y2 = Ray::new([2.0f32, 6.0, 2.0].into(), [0.0, -1.0, 0.0].into());
    let ray_z2 = Ray::new([2.0f32, 2.0, 6.0].into(), [0.0, 0.0, -1.0].into());

    assert_eq!(ray_x.intersection(&aabb), Some([1.0, 2.0, 2.0].into()));
    assert!(ray_x.intersects(&aabb));
    assert_eq!(ray_y.intersection(&aabb), Some([2.0, 1.0, 2.0].into()));
    assert!(ray_y.intersects(&aabb));
    assert_eq!(ray_z.intersection(&aabb), Some([2.0, 2.0, 1.0].into()));
    assert!(ray_z.intersects(&aabb));

    assert_eq!(ray_x2.intersection(&aabb), Some([5.0, 2.0, 2.0].into()));
    assert!(ray_x2.intersects(&aabb));
    assert_eq!(ray_y2.intersection(&aabb), Some([2.0, 5.0, 2.0].into()));
    assert!(ray_y2.intersects(&aabb));
    assert_eq!(ray_z2.intersection(&aabb), Some([2.0, 2.0, 5.0].into()));
    assert!(ray_z2.intersects(&aabb));
}

#[test]
fn test_corners() {
    let corners = Aabb2::new([-5.0f32, 5.0].into(), [5.0, 10.0].into()).to_corners();
    assert!(corners.contains(&[-5f32, 10.0].into()));
    assert!(corners.contains(&[5f32, 5.0].into()));

    let corners = Aabb3::new(
        [-20isize, 30isize, 5isize].into(),
        [10isize, -10isize, -5isize].into(),
    ).to_corners();
    assert!(corners.contains(&[-20isize, 30isize, -5isize].into()));
    assert!(corners.contains(&[10isize, 30isize, 5isize].into()));
    assert!(corners.contains(&[10isize, -10isize, 5isize].into()));
}

#[test]
fn test_bound() {
    let aabb = Aabb3::new([-5.0f32, 5.0, 0.0].into(), [5.0, 10.0, 1.0].into());
    let plane1 =
        Plane::from_point_normal([0f32, 0.0, 0.0].into(), [0f32, 0.0, 1.0].into());
    let plane2 =
        Plane::from_point_normal([-5.0f32, 4.0, 0.0].into(), [0f32, 1.0, 0.0].into());
    let plane3 =
        Plane::from_point_normal([6.0f32, 0.0, 0.0].into(), [1f32, 0.0, 0.0].into());
    assert_eq!(aabb.relate_plane(plane1), Relation::Cross);
    assert_eq!(aabb.relate_plane(plane2), Relation::In);
    assert_eq!(aabb.relate_plane(plane3), Relation::Out);
}

#[test]
fn test_aab3_should_not_intersect() {
    use collision::Discrete;
    let a = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());
    let b = Aabb3::new([15., 15., 15.].into(), [25., 25., 25.].into());
    assert!(!a.intersects(&b));
}

#[test]
fn test_aab3_should_intersect() {
    use collision::Discrete;
    let a = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());
    let b = Aabb3::new([5., 5., 5.].into(), [15., 15., 15.].into());
    assert!(a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_x() {
    use collision::Discrete;
    let a = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());
    let b = Aabb3::new([5., 11., 11.].into(), [15., 13., 13.].into());
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_y() {
    use collision::Discrete;
    let a = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());
    let b = Aabb3::new([11., 5., 11.].into(), [15., 13., 13.].into());
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb3_should_not_intersect_overlap_z() {
    use collision::Discrete;
    let a = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());
    let b = Aabb3::new([11., 11., 5.].into(), [15., 13., 13.].into());
    assert!(!a.intersects(&b));
}

#[test]
fn test_aabb2_contains_point2() {
    let aabb = Aabb2::new([0., 0.].into(), [10., 10.].into());

    let inside = [2., 2.].into();
    let outside = [11., 11.].into();

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));

    let aabb = Aabb2::<usize>::new([0, 0].into(), [10, 10].into());

    let inside = [2, 2].into();
    let outside = [11, 11].into();

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
}

#[test]
fn test_aabb2_contains_line2() {
    let aabb = Aabb2::new([0., 0.].into(), [10., 10.].into());

    let inside = Line2::new([1., 1.].into(), [2., 2.].into());
    let inside_out = Line2::new([1., 1.].into(), [12., 12.].into());
    let outside = Line2::new([11., 11.].into(), [12., 12.].into());

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb2_contains_aabb2() {
    let aabb = Aabb2::new([0., 0.].into(), [10., 10.].into());

    let inside = Aabb2::new([1., 1.].into(), [2., 2.].into());
    let inside_out = Aabb2::new([1., 1.].into(), [12., 12.].into());
    let outside = Aabb2::new([11., 11.].into(), [12., 12.].into());

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_point3() {
    let aabb = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());

    let inside = [3., 3., 3.].into();
    let outside = [11., 11., 11.].into();

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));

    let aabb = Aabb3::<usize>::new([0, 0, 0].into(), [10, 10, 10].into());

    let inside = [3, 3, 3].into();
    let outside = [11, 11, 11].into();

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
}

#[test]
fn test_aabb3_contains_line3() {
    let aabb = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());

    let inside = Line3::new([1., 1., 1.].into(), [2., 2., 2.].into());
    let inside_out = Line3::new([1., 1., 1.].into(), [12., 12., 12.].into());
    let outside = Line3::new([11., 11., 11.].into(), [12., 12., 12.].into());

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_aabb3() {
    let aabb = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());

    let inside = Aabb3::new([1., 1., 1.].into(), [2., 2., 2.].into());
    let inside_out = Aabb3::new([1., 1., 1.].into(), [12., 12., 12.].into());
    let outside = Aabb3::new([11., 11., 11.].into(), [12., 12., 12.].into());

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_aabb3_contains_sphere() {
    let aabb = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());

    let inside = Sphere {
        center: [5., 5., 5.].into(),
        radius: 1.,
    };

    let inside_out = Sphere {
        center: [5., 5., 5.].into(),
        radius: 10.,
    };
    let outside = Sphere {
        center: [20., 20., 20.].into(),
        radius: 1.,
    };

    assert!(aabb.contains(&inside));
    assert!(!aabb.contains(&outside));
    assert!(!aabb.contains(&inside_out));
}

#[test]
fn test_ray_aabb2_parallel() {
    let aabb = Aabb2::new([5., 5.].into(), [10., 10.].into());

    let ray = Ray2::new([2., 0.].into(), Vector2::new(0., 1.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray2::new([0., 2.].into(), Vector2::new(1., 0.));
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());
}

#[test]
fn test_ray_aabb3_parallel() {
    let aabb = Aabb3::new([5., 5., 5.].into(), [10., 10., 10.].into());

    let ray = Ray3::new([2., 0., 2.].into(), [0., 1., 0.].into());
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray3::new([0., 2., 2.].into(), [1., 0., 0.].into());
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());

    let ray = Ray3::new([2., 2., 0.].into(), [0., 0., 1.].into());
    assert!(!ray.intersects(&aabb));
    assert!(ray.intersection(&aabb).is_none());
}

#[test]
fn test_aabb2_union_aabb2() {
    let base = Aabb2::new([0., 0.].into(), [10., 10.].into());

    let inside = Aabb2::new([2., 2.].into(), [5., 5.].into());
    let outside = Aabb2::new([12., 12.].into(), [15., 15.].into());
    let inside_out = Aabb2::new([2., 2.].into(), [12., 12.].into());

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb2::new([0., 0.].into(), [15., 15.].into()),
        base.union(&outside)
    );
    assert_eq!(
        Aabb2::new([0., 0.].into(), [12., 12.].into()),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb3_union_aabb3() {
    let base = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());

    let inside = Aabb3::new([2., 2., 2.].into(), [5., 5., 5.].into());
    let outside = Aabb3::new([12., 12., 12.].into(), [15., 15., 15.].into());
    let inside_out = Aabb3::new([2., 2., 2.].into(), [12., 12., 12.].into());

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb3::new([0., 0., 0.].into(), [15., 15., 15.].into()),
        base.union(&outside)
    );
    assert_eq!(
        Aabb3::new([0., 0., 0.].into(), [12., 12., 12.].into()),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb3_union_sphere() {
    let base = Aabb3::new([0., 0., 0.].into(), [10., 10., 10.].into());

    let inside = Sphere {
        center: [5., 5., 5.].into(),
        radius: 1.,
    };
    let outside = Sphere {
        center: [14., 14., 14.].into(),
        radius: 1.,
    };
    let inside_out = Sphere {
        center: [8., 8., 8.].into(),
        radius: 4.,
    };

    assert_eq!(base, base.union(&inside));
    assert_eq!(
        Aabb3::new([0., 0., 0.].into(), [15., 15., 15.].into()),
        base.union(&outside)
    );
    assert_eq!(
        Aabb3::new([0., 0., 0.].into(), [12., 12., 12.].into()),
        base.union(&inside_out)
    );
}

#[test]
fn test_aabb2_surface_area() {
    let aabb = Aabb2::new([0., 0.].into(), [10., 20.].into());
    assert_eq!(200., aabb.surface_area());
}

#[test]
fn test_aabb3_surface_area() {
    let aabb = Aabb3::new([0., 0., 0.].into(), [10., 20., 30.].into());
    // 2 * (x*y + x*z + y*z)
    assert_eq!(2200., aabb.surface_area());
}

#[test]
fn test_aabb2_transform() {
    let aabb = Aabb2::new([0., 0.].into(), [10., 20.].into());
    let transform = cgmath::Decomposed::<Vector2<f32>, cgmath::Basis2<f32>> {
        disp: Vector2::new(5., 3.),
        scale: 1.,
        rot: cgmath::Rotation2::from_angle(cgmath::Rad(0.)),
    };
    assert_eq!(
        Aabb2::new([5., 3.].into(), [15., 23.].into()),
        aabb.transform(&transform)
    );
}

#[test]
fn test_aabb3_transform() {
    let aabb = Aabb3::new([0., 0., 0.].into(), [10., 20., 30.].into());
    let transform = cgmath::Decomposed::<Vector3<f32>, cgmath::Basis3<f32>> {
        disp: [5., 3., 1.].into(),
        scale: 1.,
        rot: cgmath::Rotation3::from_angle_z(cgmath::Rad(0.)),
    };
    assert_eq!(
        Aabb3::new([5., 3., 1.].into(), [15., 23., 31.].into()),
        aabb.transform(&transform)
    );
}
