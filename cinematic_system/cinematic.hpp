/******************************************************************************/
/*
  File:   cinematic.hpp
  Author: Braxton DeHate
  Brief:
    This file contains the interface for an object encapsulating the
    necessary data and relevant logic for smooth camera
    animations from keyframes, utilizing cubic bezier curves and spherical
    quadrangle interpolation (SQUAD via DirectX). This will allow developers
    a simple workflow which requires no work beyond positioning the camera at
    each desired location and orientation to create a constant speed and
    continuous direction animation.
*/
/******************************************************************************/
#pragma once
#include <winrt/base.h>
#include "turbo/common/math.hpp"
#include "turbo/common/types.hpp"
#include "turbo/core/ecs/world.hpp"
#include "turbo/core/ecs/system.hpp"

using namespace DirectX::SimpleMath;
using namespace Turbo::Common;
using namespace Types;

namespace Turbo::Graphics::Cinematic
{
  class Cinematic
  {
    struct CurvePoint;
  public:
    class Keyframe;
    static inline struct GlobalConfig
    {
      int lengthCalculationSegmentsPerKeyframe = 50;
      float reparametrizationPrecision = 10;
    } globalConfig;

    struct LocalConfig
    {
      bool smoothPosition = true;
      bool smoothRotation = true;
      bool interpolateRotation = true;
      bool useOnMainMenu = false;
    } localConfig;


    void add_keyframe(Vector3 position, Quaternion rotation);
    void delete_last_keyframe();
    void clear_keyframes();

    void recalculate_control_points();

    void recalculate_lengths();

    void debug_draw();

    std::pair<Vector3, Quaternion> update(float timeSinceStarted);

    float length();

    String name();
    void name(String newName);

    float velocity();
    void velocity(float newVelocity);

    std::vector<Keyframe> const& get_keyframes() const { return keyframes_; }

    class Keyframe
    {
    public:
      Keyframe();

      // Defined to disambiguate which control point is specified.
      // Other solution is to use a bool, which can be ambiguous
      enum ControlPoint
      {
        Before,
        After
      };

      void set_position(Vector3 newPos);
      Vector3 get_position();

      void set_rotation(Quaternion rotation);
      Quaternion get_rotation();

      void set_control_point(Vector3 position, ControlPoint controlPoint);
      Vector3 get_control_point(ControlPoint controlPoint);

      float get_length_to();
      void set_length_to(float length);

      Array<CurvePoint>& get_reparam_points();

    private:
      Array<CurvePoint> reparamPoints_;

      Vector3 position_;
      Quaternion rotation_;

      Vector3 controlPointBefore_;
      Vector3 controlPointAfter_;

      float lengthTo_;
    };

  private:
    Vector3 interpolate_position(UInt32 latterIndex, float t);
    Vector3 interpolate_position_reparameterized(UInt32 latterIndex, float t);

    Vector3 interpolate_bezier(float t, Vector3 s, Vector3 c1, Vector3 c2, Vector3 e);
    Vector3 get_bezier_derivative(float t, Vector3 s, Vector3 c1, Vector3 c2, Vector3 e);

    float length_to(UInt32 index);
    float length_to(UInt32 index, float t);

    void regen_all_control_points();
    void regen_control_points(UInt32 index);

    Array<CurvePoint> const& get_reparam_points();
    void generate_reparam_points(float precision);
    void generate_reparam_points(float precision, Size index);

    std::vector<Keyframe> keyframes_;
    String name_;

    float velocity_ = 10.0f;
    float totalLength_;

    struct CurvePoint
    {
      Vector3 position_;
      float time_;
    };
  };
}
