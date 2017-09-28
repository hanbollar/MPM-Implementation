
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "Math.h"

template <typename T>
class Camera {

    T _azimuth = 0;
    T _altitude = 0;
    T _radius = 10;
    T _ratio = 1;
    T _fov = static_cast<T>(kPI / 2);
    T _near = static_cast<T>(0.1);
    T _far = static_cast<T>(1000);

    Eigen::Matrix<T, 3, 1> _center;
    mutable Eigen::Matrix<T, 3, 1> _eyeDir, _up, _right;
    mutable Eigen::Matrix<T, 4, 4> _view;
    mutable Eigen::Matrix<T, 4, 4> _projection;
    mutable bool _viewDirty = true;
    mutable bool _projectionDirty = true;

    void Recalculate() const {
        if (_viewDirty) {
            Eigen::AngleAxis<T> r1(_altitude, Eigen::Matrix<T, 3, 1> {0, 0, 1} );
            Eigen::AngleAxis<T> r2(_azimuth, Eigen::Matrix<T, 3, 1> {0, 1, 0} );

            _eyeDir = r2 * r1 * Eigen::Matrix<T, 3, 1>{ 1, 0, 0 };
            Eigen::Matrix<T, 3, 1> eyePos = _center + _eyeDir * _radius;

            _right = (-_eyeDir).cross(Eigen::Matrix<T, 3, 1> { 0, 1, 0 }).normalized();
            _up = _right.cross(-_eyeDir);

            T data[] = {
                _right(0, 0), _up(0, 0), _eyeDir(0, 0), 0,
                _right(1, 0), _up(1, 0), _eyeDir(1, 0), 0,
                _right(2, 0), _up(2, 0), _eyeDir(2, 0), 0,
                -_right.dot(eyePos), -_up.dot(eyePos), -_eyeDir.dot(eyePos), 1,
            };

            _view = Eigen::Matrix<T, 4, 4>(data);
        }

        if (_projectionDirty) {
            T mat00 = static_cast<T>(1 / (_ratio * std::tan(0.5 * _fov)));
            T mat11 = static_cast<T>(1 / tan(0.5 * _fov));
            T mat22 = static_cast<T>(-(_near + _far) / (_far - _near));
            T mat32 = static_cast<T>(-(2 * _near * _far) / (_far - _near));

            T data[] = {
                mat00,     0,     0,     0,
                0    , mat11,     0,     0,
                0    ,     0, mat22, mat32,
                0    ,     0,    -1,     0,
            };

            _projection = Eigen::Matrix<T, 4, 4>(data).transpose();
        }

        _viewDirty = false;
        _projectionDirty = false;
    }

    public:
        Camera() {

        }

        void SetAspectRatio(T ratio) {
            _ratio = ratio;
            _projectionDirty = true;
        }

        void SetFieldOfView(T fov) {
            _fov = fov;
            _projectionDirty = true;
        }

        void SetNearFar(T n, T f) {
            _near = n;
            _far = f;
            _projectionDirty = true;
        }

        void Rotate(T dAzimuth, T dAltitude) {
            _azimuth = static_cast<T>(std::fmod(_azimuth + dAzimuth, 2 * kPI));
            _altitude = _altitude + dAltitude;

            static constexpr T kHalfPI = static_cast<T>(kPI * 49.0 / 100.0);
            _altitude = _altitude < -kHalfPI ? -kHalfPI : _altitude;
            _altitude = _altitude > kHalfPI ? kHalfPI : _altitude;
            _viewDirty = true;
        }

        void Pan(T dX, T dY) {
            this->Recalculate();
            _center += _right * dX * _radius + _up * dY * _radius;
            _viewDirty = true;
        }

        void Zoom(T factor) {
            _radius = _radius * std::exp(-factor);
            _viewDirty = true;
        }

        void LookAt(const Eigen::Matrix<T, 3, 1> center) {
            _center = center;
            _viewDirty = true;
        }

        void SetDistance(T distance) {
            _radius = distance;
            _viewDirty = true;
        }

        const Eigen::Matrix<T, 4, 4>& GetProjection() const {
            if (_projectionDirty) {
                this->Recalculate();
            }

            return _projection;
        }

        const Eigen::Matrix<T, 4, 4>& GetView() const {
            if (_viewDirty) {
                this->Recalculate();
            }

            return _view;
        }

        Eigen::Matrix<T, 4, 4> GetViewProjection() const {
            return this->GetProjection() * this->GetView();
        }

        const Eigen::Matrix<T, 3, 1>& GetPosition() const {
            return _center;
        }
    };
