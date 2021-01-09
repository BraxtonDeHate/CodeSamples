/******************************************************************************/
/*
  File:   cinematic.cpp
  Author: Braxton DeHate
  Brief:
    This file contains the implementation for an object encapsulating the
    necessary data and relevant logic for smooth camera
    animations from keyframes, utilizing cubic bezier curves and spherical
    quadrangle interpolation (SQUAD via DirectX). This will allow developers
    a simple workflow which requires no work beyond positioning the camera at
    each desired location and orientation to create a constant speed and
    continuous direction animation.
*/
/******************************************************************************/
#include "pch.h"
#include "turbo/graphics/cinematic.hpp"
#include "turbo/graphics/debug-draw.hpp"
#include "turbo/common/math.hpp"

namespace Turbo::Graphics::Cinematic
{
  // Adds a keyframe to the cinematic.
  // Param - position: World position of the keyframe
  // Param - rotation: Quaternion rotation of the camera at the keyframe
  void Cinematic::add_keyframe(Vector3 position, Quaternion rotation)
  {
    keyframes_.push_back(Keyframe());
    keyframes_.back().set_position(position);
    keyframes_.back().set_rotation(rotation);
    
    regen_all_control_points();
  }

  // Pops the last keyframe from the cinematic.
  void Cinematic::delete_last_keyframe()
  {
    if(!keyframes_.empty())
      keyframes_.pop_back();

    regen_all_control_points();
  }

  // Removes all keyframes from the cinematic.
  void Cinematic::clear_keyframes()
  {
    keyframes_.clear();
  }

  // Calculates a position and orientation from the time into the animation.
  // Update the active camera with this function every frame to play the
  // cinematic.
  // Param - timeSinceStarted: Time into the cinematic
  // Return: The intended position and rotation of the camera at the input
  //  time. Takes the set speed of the animation into account.
  std::pair<Vector3, Quaternion> Cinematic::update(Float timeSinceStarted)
  {
    timeSinceStarted *= velocity_;

    for(Int32 keyframeIndex = 1; keyframeIndex < keyframes_.size(); keyframeIndex++)
    {
      Float distance = keyframes_[keyframeIndex].get_length_to();

      // Check if we are between the right keyframes.
      // If not between the correct keyframes, reduce the time and check the next one.
      if(timeSinceStarted >= distance)
      {
        timeSinceStarted -= distance;
      }
      else
      {
        // Portion of time between adjacent keyframes elapsed
        Float factor = timeSinceStarted / distance;

        Vector3 position;
        if(localConfig.smoothPosition)
        {
          // Curve based interpolation
          position = interpolate_position_reparameterized(keyframeIndex, factor);
        }
        else
        {
          // Linear interpolation
          position = Vector3::Lerp(
            keyframes_[keyframeIndex - 1].get_position(),
            keyframes_[keyframeIndex].get_position(),
            factor);
        }

        Quaternion rotation;
        if(!localConfig.interpolateRotation)
        {
          // No rotation interpolation (Used for pans)
          rotation = keyframes_[0].get_rotation();
        }
        else if(localConfig.smoothRotation)
        {
          // Spherical Quadrangle (SQUAD) interpolation
          Quaternion rot0 = keyframes_[keyframeIndex - 1].get_rotation();
          Quaternion rot1 = keyframes_[keyframeIndex].get_rotation();

          // Clamp indices for outer quaternions to valid values.
          Int32 q0index = (keyframeIndex - 2 < 0) ? 0 : keyframeIndex - 2;
          Int32 q3index = (keyframeIndex + 1 >= keyframes_.size()) ?
            static_cast<Int32>(keyframes_.size()) - 1 :
            keyframeIndex + 1;
          // Calculate squad helper quaternions 
          Quaternion a, b, c;
          Quaternion::SquadSetup(a, b, c,
            keyframes_[q0index].get_rotation(),
            keyframes_[keyframeIndex - 1].get_rotation(),
            keyframes_[keyframeIndex].get_rotation(),
            keyframes_[q3index].get_rotation());
          rotation = Quaternion::Squad(keyframes_[keyframeIndex - 1].get_rotation(),
            a, b, c, factor);
        }
        else
        {
          // spherical (linear in angle space) implementation of rotation
          rotation = Quaternion::Slerp(keyframes_[keyframeIndex - 1].get_rotation(),
            keyframes_[keyframeIndex].get_rotation(), factor);
        }
        return std::pair<Vector3, Quaternion>(position, rotation);
      }
    }
    return std::pair<Vector3, Quaternion>();
  }

  // Interpolates between two keyframes on the connecting bezier curve.
  // Speed is not guaranteed to be constant.
  // Param - latterIndex: index of the end keyframe associated with the curve
  //  to be interpolated across
  // Param - t = [0, 1]: interpolation factor for the referenced curve
  Vector3 Cinematic::interpolate_position(UInt32 latterIndex, Float t)
  {
    Vector3 s = keyframes_[latterIndex - 1].get_position();
    Vector3 c1 = keyframes_[latterIndex - 1].get_control_point(Keyframe::After);
    Vector3 c2 = keyframes_[latterIndex].get_control_point(Keyframe::Before);
    Vector3 e = keyframes_[latterIndex].get_position();

    return interpolate_bezier(t, s, c1, c2, e);
  }

  // Interpolates between two keyframes on the connecting bezier curve.
  // Speed will be typically be virutally constant, based on the number of
  // reparam points allowed.
  // Param - latterIndex: index of the end keyframe associated with the curve
  //  to be interpolated across
  // Param - t = [0, 1]: interpolation factor for the referenced curve
  Vector3 Cinematic::interpolate_position_reparameterized(UInt32 latterIndex, Float t)
  {
    Array<CurvePoint>& points = keyframes_[latterIndex].get_reparam_points();
    if(points.size() > 0)
    {
      // Dummy curve point for the algorithm
      CurvePoint dummyPoint{ Vector3(), t };
      // Find first point which comes at a time later than t
      Array<CurvePoint>::iterator findResult = std::upper_bound(points.begin(), points.end(),
        dummyPoint, [](CurvePoint const& p1, CurvePoint const& p2) { return p1.time_ < p2.time_; });

      // Case: all points in the vector are less than t. Need to interpolate between
      // the last point and the latter keyframe
      if(findResult == points.end())
      {
        // Local t: [0, 1] parameter between t1 and t2
        // (t - t1) / (t2 - t1)
        CurvePoint const& p1 = points[points.size() - 1];
        Float localt = (t - p1.time_) / (1.0f - p1.time_);
        // Lerp between reparam points
        Vector3 result = p1.position_ * (1 - localt) +
          keyframes_[latterIndex].get_position() * localt;
        return result;
      }
      // Case: t fits in between two existing points
      else
      {
        // Local t: [0, 1] parameter between t1 and t2
        // (t - t1) / (t2 - t1)
        CurvePoint const& p1 = *(findResult - 1);
        CurvePoint const& p2 = *findResult;
        Float localt = (t - p1.time_) / (p2.time_ - p1.time_);
        // Lerp between reparam points
        Vector3 result = p1.position_ * (1 - localt) + p2.position_ * localt;

        return result;
      }
    }
    // Case: only one point
    else
    {
      Vector3 s = keyframes_[latterIndex - 1].get_position();
      Vector3 e = keyframes_[latterIndex].get_position();
      // Only one point, which should be located on the previous keyframe.
      Vector3 result = s * (1 - t) + e * t;

      return result;
    }
  }

  // Evaluates cubic bezier curve.
  // Param - t: Interpolation parameter
  // Param - s: Start point of the curve
  // Param - c1: Control point 1
  // Param - c2: Control point 2
  // Param - e: End point of the curve
  // Return: Evaluation result
  Vector3 Cinematic::interpolate_bezier(Float t, Vector3 s, Vector3 c1, Vector3 c2, Vector3 e)
  {
    // Interpolating basis: Cubic Bernstein
    //  (1-t)^3 + 3t(1-t)^2 + 3t^2(1-t) + t^3
    // With control points:
    //  S = start, E = end, C1 = Start's cp, C2 = End's cp
    //  S * (1-t)^3 + C1 * 3t(1-t)^2 + C2 * 3t^2(1-t) + E * t^3
    return s * (1 - t)*(1 - t)*(1 - t) + c1 * 3 * t*(1 - t)*(1 - t) + c2 * 3 * t*t*(1 - t) + e * t*t*t;
  }

  // Updates the lengthTo field for each keyframe.
  void Cinematic::recalculate_lengths()
  {
    totalLength_ = 0;
    for(Int32 i = 1; i < keyframes_.size(); i++)
    {
      Float length = length_to(i);
      keyframes_[i].set_length_to(length);
      totalLength_ += length;
    }
  }

  // Return: Distance between all keyframes divided by velocity
  Float Cinematic::length()
  {
    return totalLength_ / velocity_;
  }

  // Calculates the world distance length of a curve.
  // Param - index: Keyframe index of the end point of the curve.
  // Return: Distance between the keyframe at index and the keyframe at index - 1.
  Float Cinematic::length_to(UInt32 index)
  {
    // Get approximation of length by summing tangents between points on the curve
    const Int32 segments = globalConfig.lengthCalculationSegmentsPerKeyframe;
    Float totalDistance = 0;

    Vector3 lastPos, curPos;
    lastPos = interpolate_position(index, 0);
    for(Int32 segmentIndex = 1; segmentIndex <= segments; segmentIndex++)
    {
      curPos = interpolate_position(index, 1.0f / segments * segmentIndex);
      totalDistance += (curPos - lastPos).Length();
      lastPos = curPos;
    }

    return totalDistance;
  }

  // Calcuates the distance between the start of a segment (ending at keyframe index)
  // and the point on the segment represented by parameter t.
  // Param - index: Keyframe index of the end point of the curve.
  // Param - t: Interpolation parameter of the point to use.
  // Return: Distance between the keyframe at index - 1 and point represented by t.
  Float Cinematic::length_to(UInt32 index, Float t)
  {
    // Get approximation of length by summing tangents between points on the curve
    Int32 segments = static_cast<Int32>(globalConfig.lengthCalculationSegmentsPerKeyframe * t);
    if (segments == 0)
      segments = 1;
    Float totalDistance = 0;

    Vector3 lastPos, curPos;
    lastPos = interpolate_position(index, 0);
    for (Int32 i = 1; i <= segments; i++)
    {
      curPos = interpolate_position(index, t / segments * i);
      totalDistance += (curPos - lastPos).Length();
      lastPos = curPos;
    }

    return totalDistance;
  }

  // Getter for the cinematic's name.
  // Return: Name field of the cinematic.
  String Cinematic::name()
  {
    return name_;
  }

  // Setter for the cinematic's name.
  // Param - newName: New name for the cinematic.
  void Cinematic::name(String newName)
  {
    name_ = newName;
  }

  // Debug draws the cinematic's curve, keyframes, and control points for one frame.
  void Cinematic::debug_draw()
  {
    // Draw segments
    for(Int32 i = 1; i < keyframes_.size(); i++)
    {
      for(Int32 factor = 0; factor < 10; factor++)
      {
        DebugDraw::Line line;
        Vector3 pos = interpolate_position(i, 0.1f * factor);
        line.p1 = Vector4(pos.x, pos.y, pos.z, 1.0f);
        pos = interpolate_position(i, 0.1f * factor + 0.1f);
        line.p2 = Vector4(pos.x, pos.y, pos.z, 1.0f);
        DebugDraw::Painter::draw_line(0, line, Vector4(0.0, 0.9, 0.0, 1.0f));
      }
    }

    // Draw keyframes, control points
    for(Int32 i = 0; i < keyframes_.size(); i++)
    {

      constexpr Vector4 halfSize = Vector4(0.1f);
      constexpr Vector4 keyframeColor = Vector4(0.9f, 0.0f, 0.0f, 1.0f);
      constexpr Vector4 controlpointColor = Vector4(0.9f, 1.0f, 0.0f, 1.0f);


      Vector3 keyframePos = keyframes_[i].get_position();
      DebugDraw::Painter::draw_aabb(0, keyframePos - halfSize,
        keyframePos + halfSize, keyframeColor);

      Vector3 controlpointPos = keyframes_[i].get_control_point(Keyframe::Before);
      if(i != 0)
        DebugDraw::Painter::draw_aabb(0, controlpointPos - halfSize,
          controlpointPos + halfSize, controlpointColor);

      controlpointPos = keyframes_[i].get_control_point(Keyframe::After);
      if(i != keyframes_.size() - 1)
        DebugDraw::Painter::draw_aabb(0, controlpointPos - halfSize,
          controlpointPos + halfSize, controlpointColor);

      // Draw reparam points
      Array<CurvePoint>& points = keyframes_[i].get_reparam_points();
      for (Int32 j = 0; j < points.size(); j++)
      {
        constexpr Vector4 halfSize = Vector4(0.05f);
        constexpr Vector4 reparamColor = Vector4(0.25f, 0.0f, 0.75f, 1.0f);
        Vector3 point = points[j].position_;
        DebugDraw::Painter::draw_aabb(0, point - halfSize, point + halfSize, reparamColor);
      }
    }
  }

  // Calculates the control points and reparameterization points from the keyframes.
  void Cinematic::regen_all_control_points()
  {
    if (keyframes_.empty())
      return;

    //inner keyframes
    if (keyframes_.size() > 2)
    {
      for (UInt32 i = static_cast<UInt32>(keyframes_.size()) - 2; i > 0; i--)
      {
        regen_control_points(i);
      }
    }

    // First keyframe
    regen_control_points(0);
    // Last keyframe
    if (keyframes_.size() > 1)
    {
      regen_control_points(static_cast<UInt32>(keyframes_.size()) - 1);
    }

    recalculate_lengths();

    generate_reparam_points(globalConfig.reparametrizationPrecision);
  }

  // Calculates the control points for the specified keyframe.
  // The algorithm for control point calculation is sourced from:
  // https://www.scss.tcd.ie/publications/tech-reports/reports.94/TCD-CS-94-18.pdf
  // Param - index: Keyframe index to calculate for.
  void Cinematic::regen_control_points(UInt32 index)
  {
    Keyframe& keyframe = keyframes_[index];

    // Special case for only keyframe
    if (keyframes_.size() == 1)
    {
      keyframe.set_control_point(keyframe.get_position(), Keyframe::After);
      keyframe.set_control_point(keyframe.get_position(), Keyframe::Before);
    }
    else
    {
      // Special case for first keyframe
      if (index == 0)
      {
        Keyframe& nextKeyframe = keyframes_[index + 1];
        Vector3 intermediate = Vector3::Lerp(nextKeyframe.get_position(),
          nextKeyframe.get_control_point(Keyframe::Before),
          3.0f / 2.0f);

        Vector3 controlPoint = Vector3::Lerp(keyframe.get_position(),
          intermediate, 2.0f / 3.0f);

        keyframe.set_control_point(controlPoint, Keyframe::After);
      }
      // Special case for last keyframe
      else if (index == keyframes_.size() - 1)
      {
        Keyframe& prevKeyframe = keyframes_[index - 1];

        Vector3 intermediate = Vector3::Lerp(prevKeyframe.get_position(),
          prevKeyframe.get_control_point(Keyframe::After), 3.0f / 2.0f);
        Vector3 controlPoint = Vector3::Lerp(keyframe.get_position(),
          intermediate, 2.0f / 3.0f);

        keyframe.set_control_point(controlPoint, Keyframe::Before);
      }
      // Inner keyframes
      else
      {
        Keyframe& nextKeyframe = keyframes_[index + 1];
        Keyframe& prevKeyframe = keyframes_[index - 1];
        Vector3 r = Vector3::Lerp(prevKeyframe.get_position(),
          keyframe.get_position(), 2.0f);
        Vector3 t = Vector3::Lerp(r, nextKeyframe.get_position(), 0.5f);
        Vector3 finalPoint = Vector3::Lerp(keyframe.get_position(), t, 1.0f / 3.0f);
        keyframe.set_control_point(finalPoint, Keyframe::After);
        keyframe.set_control_point(keyframe.get_position() * 2 - finalPoint, Keyframe::Before);;
      }
    }
  }

  // Getter for a keyframe's reparameterized points.
  Array<Cinematic::CurvePoint>& Cinematic::Keyframe::get_reparam_points()
  {
    return reparamPoints_;
  }

  // Generates time points along the current cinematic for
  // even parameterization. Clears the current list.
  // Param - precision: Number of reparam points to use per world unit of curve.
  void Cinematic::generate_reparam_points(Float precision)
  {
    for(Int32 i = 1; i < keyframes_.size(); i++)
    {
      generate_reparam_points(precision, i);
    }
  }

  // Generate reparameterized points along the segment leading to keyframe 'index'
  // and adds them to the keyframe's vector.
  // Param - precision: Number of reparam points to use per world unit of curve.
  // Param - index: Keyframe index of the keyframe at the end of the curve.
  void Cinematic::generate_reparam_points(Float precision, Size index)
  {
    // Bounds check
    if (index == 0 || index >= keyframes_.size())
      return;

    // Get reference to the array we will fill with the points.
    Array<CurvePoint>& points = keyframes_[index].get_reparam_points();
    points.clear();
    // Get length of the referenced segment
    Float segmentLength = keyframes_[index].get_length_to();

    Int32 pointCount = static_cast<Int32>(precision * segmentLength);
    // Distance between points in t space
    Float pointDelta = static_cast<Float>(1.0f / pointCount);
    for(Int32 i = 0; i <= pointCount; i++)
    {
      // Calculate the points 
      Vector3 pointPos = interpolate_position(static_cast<UInt32>(index), pointDelta * i);
      Float pointDist = length_to(static_cast<UInt32>(index), pointDelta * i);

      points.push_back({ pointPos, pointDist / segmentLength });
    }
  }

  // Getter for the cinematic's velocity.
  Float Cinematic::velocity()
  {
    return velocity_;
  }

  // Setter for the cinematic's velocity.
  void Cinematic::velocity(Float newVelocity)
  {
    velocity_ = newVelocity;
  }

  // Keyframe constructor
  Cinematic::Keyframe::Keyframe() :
    position_(Vector3()), rotation_(Quaternion()), controlPointBefore_(Vector3()),
    lengthTo_(0.0f), controlPointAfter_(Vector3())
  {
  }

  // Gets control point position.
  // Param - controlPoint: Whether to return the connected control point that comes
  //  before or after the keyframe.
  // Return: One of the two control points connected to this keyframe.
  Vector3 Cinematic::Keyframe::get_control_point(
    ControlPoint controlPoint)
  {
    if(controlPoint == After)
    {
      // Convert before control point to the after control point
      return controlPointAfter_;
    }
    else
    {
      return controlPointBefore_;
    }
  }

  // Sets control point position.
  // Param - position: The new control point position.
  // Param - controlPoint: Whether to set the connected control point that comes
  //  before or after the keyframe.
  void Cinematic::Keyframe::set_control_point(Vector3 position,
    ControlPoint controlPoint)
  {
    if(controlPoint == After)
    {
      // Convert position to opposite control point
      controlPointAfter_ = position;
    }
    else
    {
      controlPointBefore_ = position;
    }
  }

  // Sets the position of the keyframe.
  void Cinematic::Keyframe::set_position(Vector3 newPos)
  {
    // Move the control points by the delta movement
    Vector3 delta = newPos - position_;
    controlPointBefore_ = controlPointBefore_ + delta;
    controlPointAfter_ = controlPointAfter_ + delta;

    position_ = newPos;
  }

  // Getter for the keyframe position.
  Vector3 Cinematic::Keyframe::get_position()
  {
    return position_;
  }

  // Getter for the keyframe rotation.
  Quaternion Cinematic::Keyframe::get_rotation()
  {
    return rotation_;
  }

  // Setter for the keyframe rotation.
  void Cinematic::Keyframe::set_rotation(Quaternion rotation)
  {
    rotation_ = rotation;
  }

  // Getter for the distance to the keyframe from the previous keyframe.
  Float Cinematic::Keyframe::get_length_to()
  {
    return lengthTo_;
  }

  // Setter for the distance to the keyframe from the previous keyframe.
  void Cinematic::Keyframe::set_length_to(Float length)
  {
    lengthTo_ = length;
  }
}
