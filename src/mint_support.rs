//! This module provides mint- rather than cg-math-based types
//! for communication with collision.

use std::fmt::Debug;
use num_traits::Num;
use num_traits::NumCast;
use std::ops::AddAssign;
use std::ops::SubAssign;
use std::ops::MulAssign;
use std::ops::DivAssign;
use std::ops::RemAssign;
use num_traits::Float;
use mint::Point2 as MintPoint2;
use mint::Point3 as MintPoint3;
use mint::Vector2 as MintVector2;
use mint::Vector3 as MintVector3;
use cgmath::Point2 as CGPoint2;
use cgmath::Point3 as CGPoint3;
use cgmath::Vector2 as CGVector2;
use cgmath::Vector3 as CGVector3;
use std::marker::PhantomData;
use crate::Line as CGLine;
use cgmath::VectorSpace;
use cgmath::EuclideanSpace;
use cgmath::Transform as CGTransform;
use crate::Aabb2 as CGAabb2;
use crate::Aabb3 as CGAabb3;
use crate::Aabb as CGAabbTrait;


/// All scalars must conform to the following
/// requirements (note: they are automatically provided
/// for the standard scalars like i32, f32, etc.)
pub trait MintBaseNum:
// Internal note: these need to be consistent
// with cgmath's BaseNum to support conversion
// We didn't re-export BaseNum so we could in future
// be independent of it.
Copy
+ Clone
+ Debug
+ Num
+ NumCast
+ PartialOrd
+ AddAssign
+ SubAssign
+ MulAssign
+ DivAssign
+ RemAssign
{
}

impl<T> MintBaseNum for T
    where
        T: Copy
        + Clone
        + Debug
        + Num
        + NumCast
        + PartialOrd
        + AddAssign
        + SubAssign
        + MulAssign
        + DivAssign
        + RemAssign,
{
}

/// Base floating point types
pub trait MintBaseFloat:
MintBaseNum
+ Float
+ approx::AbsDiffEq<Epsilon = Self>
+ approx::RelativeEq<Epsilon = Self>
+ approx::UlpsEq<Epsilon = Self>
{
}

impl<T> MintBaseFloat for T
    where
        T: MintBaseNum
        + Float
        + approx::AbsDiffEq<Epsilon = Self>
        + approx::RelativeEq<Epsilon = Self>
        + approx::UlpsEq<Epsilon = Self>,
{
}

// ************************************************************************************************

// Point traits (external)
/// This is the general definition of the mint point
pub trait MintPointTrait: Sized + Copy + Clone + Debug {
    /// The scalar base of the point (e.g. f32, i32)
    type Scalar: MintBaseNum;
    /// The type of the difference vector (usually Vector2<Scalar> or Vector4<Scalar>)
    type Diff;
    /// The type of the point (usually Point2<Scalar> or Point3<Scalar>)
    type Point;
}
impl<S> MintPointTrait for MintPoint2<S>
    where
        S: MintBaseNum
{
    type Scalar = S;
    type Diff = MintVector2<S>;
    type Point = MintPoint2<S>;
}
impl<S> MintPointTrait for MintPoint3<S>
    where
        S: MintBaseNum
{
    type Scalar = S;
    type Diff = MintVector3<S>;
    type Point = MintPoint3<S>;
}

/// The general traits of a mint vector
///
/// A vector is a direction, a point is a place.
pub trait MintVectorTrait: Sized + Copy + Clone + Debug {
    /// The scalar base of the point (e.g. f32, i32)
    type Scalar: MintBaseNum;
    /// The type of the vector trait (e.g. Vector2<f32>)
    type Diff;
}
impl<S> MintVectorTrait for MintVector2<S>
    where S: MintBaseNum,
{
    type Scalar = S;
    type Diff = MintVector2<S>;
}
impl<S> MintVectorTrait for MintVector3<S>
    where S: MintBaseNum,
{
    type Scalar = S;
    type Diff = MintVector2<S>;
}

/// This is the data structure for a mint line
#[derive(Copy, Clone, Debug)]
pub struct MintLine<S,P> {
    origin: P,
    dest: P,
    phantom_data_s: PhantomData<S>,
}
/// A 2D directed line from origin to destination:
pub type MintLine2<S> = MintLine<S, MintPoint2<S>>;
/// A 3D directed line from origin to destination:
pub type MintLine3<S> = MintLine<S, MintPoint3<S>>;
/// The trait information for a Mint Line
pub trait MintLineTrait: Sized + Copy + Clone + Debug {
    /// The scalar base of the point (e.g. f32, i32)
    type Scalar: MintBaseNum;
    /// The type of the line's direction (e.g. Vector2<Scalar>)
    type Diff: MintVectorTrait<Scalar = <Self as MintLineTrait>::Scalar>;
    /// The type of the points of the line (e.g. Point2<Scalar>)
    type Point: MintPointTrait<Scalar = <Self as MintLineTrait>::Scalar>;
    /// Create a new line from origin and destination points
    fn new<I:Sized+Into<Self::Point>>(origin: I, dest: I) -> MintLine<Self::Scalar, Self::Point> {
        MintLine {
            origin: origin.into(),
            dest: dest.into(),
            phantom_data_s: PhantomData,
        }
    }
}
impl<S> MintLineTrait for MintLine2<S>
    where
        S: MintBaseNum
{
    type Scalar = S;
    type Diff = MintVector2<S>;
    type Point = MintPoint2<S>;
}
impl<S> MintLineTrait for MintLine3<S>
    where
        S: MintBaseNum
{
    type Scalar = S;
    type Diff = MintVector3<S>;
    type Point = MintPoint3<S>;
}
impl<S,V,P> Into<CGLine<S,V,P>> for MintLine2<S>
    where
        S: MintBaseNum,
        V: VectorSpace<Scalar = S>,
        P: EuclideanSpace<Scalar = S, Diff = V> + From<MintPoint2<S>>,
{
    fn into(self) -> CGLine<S, V, P> {
        CGLine::new(self.origin.into(), self.dest.into())
    }
}
impl<S,V,P> From<CGLine<S,V,P>> for MintLine2<S>
    where
        S: MintBaseNum,
        V: VectorSpace<Scalar = S>,
        P: EuclideanSpace<Scalar = S, Diff = V> + Into<MintPoint2<S>>,
{
    fn from(other: CGLine<S, V, P>) -> Self {
        MintLine::<S, MintPoint2<S>>::new( other.origin, other.dest )
    }
}
impl<S,V,P> Into<CGLine<S,V,P>> for MintLine3<S>
    where
        S: MintBaseNum,
        V: VectorSpace<Scalar = S>,
        P: EuclideanSpace<Scalar = S, Diff = V> + From<MintPoint3<S>>,
{
    fn into(self) -> CGLine<S, V, P> {
        CGLine::new(self.origin.into(), self.dest.into())
    }
}
impl<S,V,P> From<CGLine<S,V,P>> for MintLine3<S>
    where
        S: MintBaseNum,
        V: VectorSpace<Scalar = S>,
        P: EuclideanSpace<Scalar = S, Diff = V> + Into<MintPoint3<S>>,
{
    fn from(other: CGLine<S, V, P>) -> Self {
        MintLine::<S, MintPoint3<S>>::new( other.origin, other.dest )
    }
}

/// An axis-aligned bounding box for an object
#[derive(Copy, Clone, Debug)]
pub struct MintAABBStruct<S,P> {
    /// Minimum point of the AABB
    pub min: P,
    /// Maximum point of the AABB
    pub max: P,
    phantom_data_s: PhantomData<S>,
}
/// A two-dimensional AABB, aka a rectangle.
pub type MintAabb2<S> = MintAABBStruct<S, MintPoint2<S>>;
/// A three-dimensional AABB, aka a rectangular prism.
pub type MintAabb3<S> = MintAABBStruct<S, MintPoint3<S>>;
// Conversion support:
impl<S> Into<CGAabb2<S>> for MintAabb2<S>
    where
        S: MintBaseNum,
        CGPoint2<S>: From<MintPoint2<S>>,
{
    fn into(self) -> CGAabb2<S> {
        // MintAabb2::new just converts it back into
        // CGAabb2, anyway, so we bypass this
        // and make the structure directly:
        CGAabb2::new(self.min.into(), self.max.into())
    }
}
impl<S> From<CGAabb2<S>> for MintAabb2<S>
    where
        S: MintBaseNum,
        CGPoint2<S>: Into<MintPoint2<S>>,
{
    fn from(other: CGAabb2<S>) -> Self {
        MintAabb2 {
            min: other.min.into(),
            max: other.max.into(),
            phantom_data_s: PhantomData,
        }
    }
}
impl<S> From<&MintAabb2<S>> for CGAabb2<S>
    where
        S: MintBaseNum,
        CGPoint2<S>: From<MintPoint2<S>>,
{
    fn from(other: &MintAABBStruct<S, MintPoint2<S>>) -> Self {
        CGAabb2::new(other.min.into(), other.max.into())
    }
}
impl<S> Into<CGAabb3<S>> for MintAabb3<S>
    where
        S: MintBaseNum,
        CGPoint3<S>: From<MintPoint3<S>>,
{
    fn into(self) -> CGAabb3<S> {
        // MintAabb3::new just converts it back into
        // CGAabb3, anyway, so we bypass this
        // and make the structure directly:
        CGAabb3::new(self.min.into(), self.max.into())
    }
}
impl<S> From<&MintAabb3<S>> for CGAabb3<S>
    where
        S: MintBaseNum,
        CGPoint3<S>: From<MintPoint3<S>>,
{
    fn from(other: &MintAABBStruct<S, MintPoint3<S>>) -> Self {
        CGAabb3::new(other.min.into(), other.max.into())
    }
}
impl<S> From<CGAabb3<S>> for MintAabb3<S>
    where
        S: MintBaseNum,
        CGPoint3<S>: Into<MintPoint3<S>>,
{
    fn from(other: CGAabb3<S>) -> Self {
        MintAabb3 {
            min: other.min.into(),
            max: other.max.into(),
            phantom_data_s: PhantomData,
        }
    }
}
/// Base trait describing an axis aligned bounding box.
pub trait MintAABBTrait: Sized {
    /// Scalar type
    type Scalar: MintBaseNum;

    /// Vector type
    type Diff: MintVectorTrait;

    /// Point type
    type Point: MintPointTrait;

    /// GGPoint type
    type CGPoint: EuclideanSpace;

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
            T: CGTransform<Self::CGPoint>;
}
impl<S> MintAABBTrait for MintAabb2<S>
    where S: MintBaseNum,
          MintPoint2<S>: Into<CGPoint2<S>> + From<CGPoint2<S>>,
          MintVector2<S>: Into<CGVector2<S>> + From<CGVector2<S>>,
          CGPoint2<S>: From<MintPoint2<S>>,
{
    type Scalar = S;
    type Diff = MintVector2<S>;
    type Point = MintPoint2<S>;
    type CGPoint = CGPoint2<S>;

    fn new(p1: Self::Point, p2: Self::Point) -> Self {
        // CGAabb actually does some data cleaning,
        // so we don't bypass it
        CGAabb2::new(p1.into(), p2.into()).into()
    }

    fn zero() -> Self {
        CGAabb2::zero().into()
    }

    fn min(&self) -> Self::Point {
        self.min
    }

    fn max(&self) -> Self::Point {
        self.max
    }

    fn dim(&self) -> Self::Diff {
        (CGAabb2::<S>::from(self)).dim().into()
    }

    fn volume(&self) -> Self::Scalar {
        (CGAabb2::<S>::from(self)).volume()
    }

    fn center(&self) -> Self::Point {
        (CGAabb2::<S>::from(self)).center().into()
    }

    fn grow(&self, p: Self::Point) -> Self {
        (CGAabb2::<S>::from(self)).grow(p.into()).into()
    }

    fn add_v(&self, v: Self::Diff) -> Self {
        (CGAabb2::<S>::from(self)).add_v(v.into()).into()
    }

    fn add_margin(&self, margin: Self::Diff) -> Self {
        (CGAabb2::<S>::from(self)).add_margin(margin.into()).into()
    }

    fn mul_s(&self, s: Self::Scalar) -> Self {
        (CGAabb2::<S>::from(self)).mul_s(s).into()
    }

    fn mul_v(&self, v: Self::Diff) -> Self {
        (CGAabb2::<S>::from(self)).mul_v(v.into()).into()
    }

    fn transform<T>(&self, transform: &T) -> Self where
        T: CGTransform<Self::CGPoint> {
        (CGAabb2::<S>::from(self)).transform(transform).into()
    }
}
impl<S> MintAABBTrait for MintAabb3<S>
    where S: MintBaseNum,
          MintPoint3<S>: Into<CGPoint3<S>> + From<CGPoint3<S>>,
          MintVector3<S>: Into<CGVector3<S>> + From<CGVector3<S>>,
          CGPoint3<S>: From<MintPoint3<S>>,
{
    type Scalar = S;
    type Diff = MintVector3<S>;
    type Point = MintPoint3<S>;
    type CGPoint = CGPoint3<S>;

    fn new(p1: Self::Point, p3: Self::Point) -> Self {
        // CGAabb actually does some data cleaning,
        // so we don't bypass it
        CGAabb3::new(p1.into(), p3.into()).into()
    }

    fn zero() -> Self {
        CGAabb3::zero().into()
    }

    fn min(&self) -> Self::Point {
        self.min
    }

    fn max(&self) -> Self::Point {
        self.max
    }

    fn dim(&self) -> Self::Diff {
        (CGAabb3::<S>::from(self)).dim().into()
    }

    fn volume(&self) -> Self::Scalar {
        (CGAabb3::<S>::from(self)).volume()
    }

    fn center(&self) -> Self::Point {
        (CGAabb3::<S>::from(self)).center().into()
    }

    fn grow(&self, p: Self::Point) -> Self {
        (CGAabb3::<S>::from(self)).grow(p.into()).into()
    }

    fn add_v(&self, v: Self::Diff) -> Self {
        (CGAabb3::<S>::from(self)).add_v(v.into()).into()
    }

    fn add_margin(&self, margin: Self::Diff) -> Self {
        (CGAabb3::<S>::from(self)).add_margin(margin.into()).into()
    }

    fn mul_s(&self, s: Self::Scalar) -> Self {
        (CGAabb3::<S>::from(self)).mul_s(s).into()
    }

    fn mul_v(&self, v: Self::Diff) -> Self {
        (CGAabb3::<S>::from(self)).mul_v(v.into()).into()
    }

    fn transform<T>(&self, transform: &T) -> Self where
        T: CGTransform<Self::CGPoint> {
        (CGAabb3::<S>::from(self)).transform(transform).into()
    }
}
