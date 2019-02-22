#![crate_type = "rlib"]
#![crate_type = "dylib"]
#![deny(missing_docs, trivial_casts, unsafe_code, unstable_features, unused_import_braces,
unused_qualifications)]

//! Companion library to cgmath, dealing with collision detection centric data structures and
//! algorithms.
//!
//! This crate provides useful data structures and algorithms for doing collision detection.
//! It is organized into a few distinct parts: generic geometry (ray, line, plane, frustum etc),
//! bounding volumes (AABB, OBB, Sphere etc), collision primitives and algorithms used for
//! collision detection, distance computation etc.
//!
#[macro_use]
extern crate approx;

extern crate bit_set;
extern crate cgmath;
extern crate num;
extern crate rand;

#[cfg(feature = "serde")]
#[macro_use]
extern crate serde;
#[cfg_attr(test, macro_use)]
extern crate smallvec;

use mint;
use std::marker::PhantomData;


// This is the public API of collision-rs,
// which is purely data-driven using mint types.
// Mint data types have no capabilities requirements.
// They can be converted to and from the standard linear algebra
// data types provided by cgmath (feature: ["mint"])
// and nalegbra.

// If you are not doing anything with your points, vectors
// and matrices other than calling collision-rs methods
// on them, we recommend you use mint-types in your code.

// If you are using more complex operations,
// convert your data to mint types as you pass it
// to collision-rs.

// If you depend on the capabilities for your types in a library,
// remember that anyone using your library has to
// have the EXACT SAME version of all those capabilities,
// even if the library you rely on (e.g. cgmath) has moved on.

// TODO: consistently implement traits and structs for each
// of our mint structures. The scalar and vector types are stored
// in the trait, so we can make generic traits from 2D and 3D

/// The base numeric type with partial sorting
/// Borrowed from cgmath and reduced to what we need
pub trait MintBaseNum: num::Num + Copy + Clone + std::fmt::Debug + PartialOrd {}
impl<T> MintBaseNum for T where T: num::Num + Copy + Clone + std::fmt::Debug + PartialOrd, {}

/// The base floating-point type
/// Borrowed from cgmath and reduced to what we need
pub trait MintBaseFloat: MintBaseNum + num::Float {}

/// The Point2 of our API (you can use mint::Point2 if you prefer)
pub type MintPoint2<S> = mint::Point2<S>;

/// The Point3 type of our API (you can use mint::Point3 if you prefer)
pub type MintPoint3<S> = mint::Point3<S>;

/// Minimum requirements of our point:
pub trait MintPointTrait<P>: Sized + Copy + Clone + std::fmt::Debug + Into<P>  {
    type Scalar: MintBaseNum;
    type Point: Sized + Copy + Clone + std::fmt::Debug + Into<P>;
}
impl <T,P> MintPointTrait<P> for T where T: Sized + Copy + Clone + std::fmt::Debug + Into<P>,{
    type Scalar = P;
    type Point = T;
}

// The 2d point structure we use
// for passing into and out of our library:
/// A simple line: from..to. 2D and 3D versions
/// S: the scalar type of the line's components (e.g. f32, u64)
pub type MintLine2<S> = MintLine<S, mint::Point2<S>>;

/// 3D directed line segment
pub type MintLine3<S> = MintLine<S, mint::Point3<S>>;

/// A generic directed line segment from `origin` to `dest`.
///
/// Note: don't use this generic line! Use Line2 and Line3
/// instead. (Type rules are not enforced for this structure).
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MintLine<S, T> {
    /// Origin of the line
    pub origin: T,
    /// Endpoint of the line
    pub dest: T,
    phantom_s: PhantomData<S>
}
impl <S:MintBaseNum, T> MintLine<S,T> {
    /// Create a new directed line segment from `origin` to `dest`.
    ///
    /// Use .into() to create minty values from whatever you're
    /// using in your code.
    pub fn new<I:Into<T>>(origin: I, dest: I) -> MintLine<S,T> {
        MintLine {
            origin: origin.into(),
            dest: dest.into(),
            phantom_s: PhantomData,
        }
    }
}


// Re-exports

pub use bound::*;
pub use contact::*;
pub use frustum::*;
pub use line::*;
pub use plane::Plane;
pub use ray::*;
pub use traits::*;
pub use volume::*;

/// A two-dimensional AABB, aka a rectangle.
pub type MintAabb2<S> = MintAabbStruct<S, MintPoint2<S>>;
/// A three-dimensional AABB, aka a rectangular prism.
pub type MintAabb3<S> = MintAabbStruct<S, MintPoint3<S>>;
/// A generic AABB structure. Don't use this directly!
/// Use MintAabb2 or MintAabb3 instead.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MintAabbStruct<S,T> {
    /// Minimum point of the AABB
    pub min: T,
    /// Maximum point of the AABB
    pub max:T,
    phantomdata_s: PhantomData<S>,
}
impl<S:MintBaseNum,T> MintAabbStruct<S,T> {
    /// Construct a new axis-aligned bounding box from two points.
    pub fn new<I:Into<T>>(min: I, max: I) -> MintAabbStruct<S, T> {
        MintAabbStruct {
            min: min.into(),
            max: max.into(),
            phantomdata_s: PhantomData,
        }
    }
}
impl <S:MintBaseNum> Into<Aabb2<S>> for MintAabb2<S>{
    fn into(self) -> Aabb2<S> {
        Aabb2 {
            min: self.min.into(),
            max: self.max.into(),
        }
    }
}
impl <S:MintBaseNum> Into<MintAabb2<S>> for Aabb2<S> {
    fn into(self) -> MintAabb2<S> {
        MintAabbStruct::<S, MintPoint2<S>>::new(
            self.min,
            self.max,
        )
    }
}

/// Public transform trait for mint types
///
/// We promise to support these methods if you
/// for the following types:
/// P: Into<MintPoint2<S> or Into<MintPoint3<S>
/// V: Diff vector, Into<MintVector2<S> or IntoMintVector3<S>
pub trait MintTransform<P,V>: Sized {
    /// Create an identity transformation. That is, a transformation which
    /// does nothing.
    fn one() -> Self;

    /// Create a transformation that rotates a vector to look at `center` from
    /// `eye`, using `up` for orientation.
    fn look_at(eye: P, center: P, up: V) -> Self;

    /// Transform a vector using this transform.
    fn transform_vector(&self, vec: V) -> V;

    /// Inverse transform a vector using this transform
    fn inverse_transform_vector(&self, vec: V) -> Option<V> {
        self.inverse_transform()
            .and_then(|inverse| Some(inverse.transform_vector(vec)))
    }

    /// Transform a point using this transform.
    fn transform_point(&self, point: P) -> P;

    /// Combine this transform with another, yielding a new transformation
    /// which has the effects of both.
    fn concat(&self, other: &Self) -> Self;

    /// Create a transform that "un-does" this one.
    fn inverse_transform(&self) -> Option<Self>;

    /// Combine this transform with another, in-place.
    #[inline]
    fn concat_self(&mut self, other: &Self) {
        *self = Self::concat(self, other);
    }
}

/// This is a copy of volume::aabb::Aabb trait.
/// The return and input values are using mints.
/// If that trait is modified, this needs to be
/// done as well.
pub trait MintAabb2Trait: Sized {
    /// Scalar type
    type Scalar: MintBaseNum;

    /// Vector type
    type Diff: Into<mint::Vector2<Self::Scalar>>;

    /// Point type
    type Point: Into<mint::Point2<Self::Scalar>>;

    /// Create a new AABB using two points as opposing corners.
    fn new(p1: Self::Point, p2: Self::Point) -> Self;

    /// Create a new empty AABB
    fn zero() -> Self;

    /// Return a shared reference to the point nearest to (-inf, -inf).
    fn min(&self) -> Self::Point;

    /// Return a shared reference to the point nearest to (inf, inf).
    fn max(&self) -> Self::Point;

    /// Return the dimensions of this AABB.
    #[inline]
    fn dim(&self) -> Self::Diff;

    /// Return the volume this AABB encloses.
    #[inline]
    fn volume(&self) -> Self::Scalar;

    /// Return the center point of this AABB.
    #[inline]
    fn center(&self) -> Self::Point;

    /// Returns a new AABB that is grown to include the given point.
    fn grow(&self, p: Self::Point) -> Self;

    /// Add a vector to every point in the AABB, returning a new AABB.
    #[inline]
    fn add_v(&self, v: Self::Diff) -> Self;

    /// Add a margin of the given width around the AABB, returning a new AABB.
    fn add_margin(&self, margin: Self::Diff) -> Self;

    /// Multiply every point in the AABB by a scalar, returning a new AABB.
    #[inline]
    fn mul_s(&self, s: Self::Scalar) -> Self;

    /// Multiply every point in the AABB by a vector, returning a new AABB.
    fn mul_v(&self, v: Self::Diff) -> Self;

    /// Apply an arbitrary transform to the corners of this bounding box,
    /// return a new conservative bound.
    fn transform<T>(&self, transform: &T) -> Self
        where
            T: MintTransform<Self::Point, Self::Diff>;
}
impl <S:MintBaseNum> MintAabb2Trait for MintAabb2<S> {
    type Scalar = S;
    type Diff = mint::Vector2<S>;
    type Point = mint::Point2<S>;

    fn new(p1: Self::Point, p2: Self::Point) -> Self {
        MintAabb2::<>::new(p1, p2)
    }

    fn zero() -> Self {
        let z = Self::Scalar::zero();
        let p:MintPoint2<S> = [z, z].into();
        Self::new(p, p)
    }

    fn min(&self) -> Self::Point {
        self.min
    }

    fn max(&self) -> Self::Point {
        self.max
    }

    fn dim(&self) -> Self::Diff {
        unimplemented!()
    }

    fn volume(&self) -> Self::Scalar {
        unimplemented!()
    }

    fn center(&self) -> Self::Point {
        unimplemented!()
    }

    fn grow(&self, p: Self::Point) -> Self {
        unimplemented!()
    }

    fn add_v(&self, v: Self::Diff) -> Self {
        unimplemented!()
    }

    fn add_margin(&self, margin: Self::Diff) -> Self {
        unimplemented!()
    }

    fn mul_s(&self, s: Self::Scalar) -> Self {
        unimplemented!()
    }

    fn mul_v(&self, v: Self::Diff) -> Self {
        unimplemented!()
    }

    fn transform<T>(&self, transform: &T) -> Self where
        T: MintTransform<Self::Point, Self::Diff> {
        unimplemented!()
    }
}
impl<S: MintBaseNum> Contains<MintPoint2<S>> for MintAabb2<S> {
    #[inline]
    fn contains(&self, p: &MintPoint2<S>) -> bool {
        self.min.x <= p.x && p.x < self.max.x && self.min.y <= p.y && p.y < self.max.y
    }
}
impl<S: MintBaseNum> std::fmt::Debug for MintAabb2<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "[{:?} - {:?}]", self.min, self.max)
    }
}
/// This is a copy of volume::aabb::Aabb trait.
/// The return and input values are using mints.
/// If that trait is modified, this needs to be
/// done as well.
pub trait MintAabb3Trait: Sized {
    /// Scalar type
    type Scalar: MintBaseNum;

    /// Vector type
    type Diff: Into<mint::Vector3<Self::Scalar>>;

    /// Point type
    type Point: Into<mint::Point3<Self::Scalar>>;

    /// Create a new AABB using two points as opposing corners.
    fn new(p1: Self::Point, p2: Self::Point) -> Self;

    /// Create a new empty AABB
    fn zero() -> Self;

    /// Return a shared reference to the point nearest to (-inf, -inf).
    fn min(&self) -> Self::Point;

    /// Return a shared reference to the point nearest to (inf, inf).
    fn max(&self) -> Self::Point;

    /// Return the dimensions of this AABB.
    #[inline]
    fn dim(&self) -> Self::Diff;

    /// Return the volume this AABB encloses.
    #[inline]
    fn volume(&self) -> Self::Scalar;

    /// Return the center point of this AABB.
    #[inline]
    fn center(&self) -> Self::Point;

    /// Returns a new AABB that is grown to include the given point.
    fn grow(&self, p: Self::Point) -> Self;

    /// Add a vector to every point in the AABB, returning a new AABB.
    #[inline]
    fn add_v(&self, v: Self::Diff) -> Self;

    /// Add a margin of the given width around the AABB, returning a new AABB.
    fn add_margin(&self, margin: Self::Diff) -> Self;

    /// Multiply every point in the AABB by a scalar, returning a new AABB.
    #[inline]
    fn mul_s(&self, s: Self::Scalar) -> Self;

    /// Multiply every point in the AABB by a vector, returning a new AABB.
    fn mul_v(&self, v: Self::Diff) -> Self;

    /// Apply an arbitrary transform to the corners of this bounding box,
    /// return a new conservative bound.
    fn transform<T>(&self, transform: &T) -> Self
        where
            T: MintTransform<Self::Point, Self::Diff>;
}
impl <S:MintBaseNum> MintAabb3Trait for MintAabb3<S> {
    type Scalar = S;
    type Diff = mint::Vector3<S>;
    type Point = mint::Point3<S>;

    fn new(p1: Self::Point, p2: Self::Point) -> Self {
        MintAabb3::<>::new(p1, p2)
    }

    fn zero() -> Self {
        let z = Self::Scalar::zero();
        let p:MintPoint3<S> = [z, z, z].into();
        Self::new(p, p)
    }

    fn min(&self) -> Self::Point {
        self.min
    }

    fn max(&self) -> Self::Point {
        self.max
    }

    fn dim(&self) -> Self::Diff {
        unimplemented!()
    }

    fn volume(&self) -> Self::Scalar {
        unimplemented!()
    }

    fn center(&self) -> Self::Point {
        unimplemented!()
    }

    fn grow(&self, p: Self::Point) -> Self {
        unimplemented!()
    }

    fn add_v(&self, v: Self::Diff) -> Self {
        unimplemented!()
    }

    fn add_margin(&self, margin: Self::Diff) -> Self {
        unimplemented!()
    }

    fn mul_s(&self, s: Self::Scalar) -> Self {
        unimplemented!()
    }

    fn mul_v(&self, v: Self::Diff) -> Self {
        unimplemented!()
    }

    fn transform<T>(&self, transform: &T) -> Self where
        T: MintTransform<Self::Point, Self::Diff> {
        unimplemented!()
    }
}
impl<S: MintBaseNum> Contains<MintPoint3<S>> for MintAabb3<S> {
    #[inline]
    fn contains(&self, p: &MintPoint3<S>) -> bool {
        self.min.x <= p.x && p.x < self.max.x && self.min.y <= p.y && p.y < self.max.y && self.min.z <= p.z && p.z < self.max.z
    }
}
impl<S: MintBaseNum> std::fmt::Debug for MintAabb3<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "[{:?} - {:?}]", self.min, self.max)
    }
}


pub mod prelude;
pub mod dbvt;
pub mod primitive;
pub mod algorithm;

// Modules

mod bound;
mod frustum;
mod traits;
mod plane;
mod ray;
mod line;
mod volume;
mod contact;
