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
pub struct MintLine<S,V,P> {
    origin: P,
    dest: P,
    phantom_data_s: PhantomData<S>,
    phantom_data_p: PhantomData<V>,
}
/// A 2D directed line from origin to destination:
pub type MintLine2<S> = MintLine<S, MintVector2<S>, MintPoint2<S>>;
/// A 3D directed line from origin to destination:
pub type MintLine3<S> = MintLine<S, MintVector3<S>, MintPoint3<S>>;
/// The trait information for a Mint Line
pub trait MintLineTrait: Sized + Copy + Clone + Debug {
    /// The scalar base of the point (e.g. f32, i32)
    type Scalar: MintBaseNum;
    /// The type of the line's direction (e.g. Vector2<Scalar>)
    type Diff: MintVectorTrait<Scalar = <Self as MintLineTrait>::Scalar>;
    /// The type of the points of the line (e.g. Point2<Scalar>)
    type Point: MintPointTrait<Scalar = <Self as MintLineTrait>::Scalar>;
    /// Create a new line from origin and destination points
    fn new<I:Sized+Into<Self::Point>>(origin: I, dest: I) -> MintLine<Self::Scalar, Self::Diff, Self::Point> {
        MintLine {
            origin: origin.into(),
            dest: dest.into(),
            phantom_data_s: PhantomData,
            phantom_data_p: PhantomData,
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
        MintLine::<S, MintVector2<S>, MintPoint2<S>>::new( other.origin, other.dest )
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
        MintLine::<S, MintVector3<S>, MintPoint3<S>>::new( other.origin, other.dest )
    }
}

// TODO: Implement AABB, from and to.