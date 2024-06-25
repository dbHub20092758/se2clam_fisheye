//
// Created by duan on 2022/12/20.
//

#ifndef RTSLAM_CAMERA_H
#define RTSLAM_CAMERA_H

#include <Eigen/Dense>

namespace vslam
{

    enum CameraModels{
        PINHOLE = 1,
        CATA_MEI = 2,
        FISHEYE = 3
    };


    class Camera{
    public:
        Camera(const double fx, const double fy, const double cx, const double cy,
               const double k1,const double k2,const double k3,
               const double p1,const double p2): fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1),
                                                 k2_(k2), k3_(k3), p1_(p1), p2_(p2){

        }

        ~Camera() = default;

        virtual void worldToImage(const Eigen::Vector3d &pc, Eigen::Vector2d &pixel) = 0;

        virtual void imageToWorld(const Eigen::Vector2d &pixel, Eigen::Vector3d &pc_norm_plane) = 0;

        //ceres优化用
        virtual void getParams(std::vector<double> &params) = 0;

        template<typename T>
        void distort(const T& u, const T& v, T &du, T &dv){

            T r2 = u * u + v * v;
            T r4 = r2 * r2;
            T r6 = r2 * r4;
            T x2 = u * u;
            T y2 = v * v;
            T radial_distort = k1_ * r2 + k2_ * r4 + k3_ * r6;
            T dx = 2.0 * p1_ * u * v + p2_ * (r2 + 2.0 * x2);
            T dy = 2.0 * p2_ * u * v + p1_ * (r2 + 2.0 * y2);

            T x1 = radial_distort * u + dx;
            T y1 = radial_distort * v + dy;

            du = x1;
            dv = y1;
        }

    public:
        double fx_;
        double fy_;
        double cx_;
        double cy_;
        double k1_;
        double k2_;
        double k3_;
        double p1_;
        double p2_;

        double xi_;
    };

    typedef std::shared_ptr<Camera> CameraPtr;

    class Pinhole : public Camera{
    public:
        Pinhole(const double fx, const double fy, const double cx, const double cy,
                const double k1,const double k2,const double k3,
                const double p1,const double p2)
                : Camera(fx, fy, cx, cy, k1, k2, k3, p1, p2){
        }

        ~Pinhole() = default;

        virtual void worldToImage(const Eigen::Vector3d &pc, Eigen::Vector2d &pixel) override{

            //normalize plane
            double x = pc.x() / pc.z();
            double y = pc.y() / pc.z();

            //distort
            double dx, dy;
            distort(x, y, dx, dy);

            x += dx;
            y += dy;

            pixel.x() = fx_ * x + cx_;
            pixel.y() = fy_ * y + cy_;
        }

        virtual void imageToWorld(const Eigen::Vector2d &pixel, Eigen::Vector3d &pc_norm_plane) override{

            //归一化平面
            double x = (pixel.x() - cx_) / fx_;
            double y = (pixel.y() - cy_) / fy_;

            //去畸变
            double x1 = x;
            double y1 = y;
            double dx, dy;
            const int iter = 8;
            for(int i = 0; i < iter; i++){
                distort(x1, y1, dx, dy);
                x1 = x - dx;
                y1 = y - dy;
            }

            pc_norm_plane.x() = x1;
            pc_norm_plane.y() = y1;
            pc_norm_plane.z() = 1.0;
        }

        virtual void getParams(std::vector<double> &params) override{

            params.clear();
            params.reserve(10);

            params.emplace_back(fx_);
            params.emplace_back(fy_);
            params.emplace_back(cx_);
            params.emplace_back(cy_);
            params.emplace_back(k1_);
            params.emplace_back(k2_);
            params.emplace_back(k3_);
            params.emplace_back(p1_);
            params.emplace_back(p2_);
        }

        template<typename T>
        static void worldToImage(
                const T* const params,const T &x,const T& y,const T& z,
                T &pixel_x, T &pixel_y){

            T fx = params[0];
            T fy = params[1];
            T cx = params[2];
            T cy = params[3];
            T k1 = params[4];
            T k2 = params[5];
            T k3 = params[6];
            T p1 = params[7];
            T p2 = params[8];

            //normalize plane
            T u = x / z;
            T v = y / z;

            //distort
            T r2 = u * u + v * v;
            T r4 = r2 * r2;
            T r6 = r2 * r4;
            T x2 = u * u;
            T y2 = v * v;
            T radial_distort = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
            T dx = 2.0 * p1 * u * v + p2 * (r2 + 2.0 * x2);
            T dy = 2.0 * p2 * u * v + p1 * (r2 + 2.0 * y2);

            T x1 = radial_distort * u + dx;
            T y1 = radial_distort * v + dy;

            pixel_x = fx * x1 + cx;
            pixel_y = fy * y1 + cy;
        }

    };

    class CataCamera : public Camera{
    public:
        CataCamera(const double xi,
                   const double fx, const double fy, const double cx, const double cy,
                   const double k1,const double k2,const double k3,
                   const double p1,const double p2)
                : Camera(fx, fy, cx, cy, k1, k2,k3,p1,p2){
            xi_ = xi;
        }

        ~CataCamera() = default;

        virtual void imageToWorld(const Eigen::Vector2d &pixel, Eigen::Vector3d &pc_norm_plane) override{

            double xd = (pixel.x() - cx_) / fx_;
            double yd = (pixel.y() - cy_) / fy_;

            //去畸变
            double x = xd;
            double y = yd;
            double dx, dy;
            const int iter = 8;
            for(int i = 0; i < iter; i++){
                distort(x, y, dx, dy);
                x = xd - dx;
                y = yd - dy;
            }

            //sphere
//            double lambda;
//            Eigen::Vector3d sphere;
//            if(xi_ == 1.0){
//                lambda = 2.0 / (x * x + y * y + 1.0);
//                sphere << lambda * x, lambda * y, lambda - 1.0;
//            }else{
//                lambda = (xi_ + std::sqrt(1.0 + (1.0 - xi_ * xi_) * (x * x + y * y))) / (1.0 + x * x + y *y);
//                sphere << lambda * x, lambda * y, lambda - xi_;
//            }
//
//            pc_norm_plane.x() = sphere.x() / sphere.z();
//            pc_norm_plane.y() = sphere.y() / sphere.z();
//            pc_norm_plane.z() = 1.0;

            //projective
            double r2 = x * x + y * y;
            pc_norm_plane << x, y, 1.0 - xi_ * (1.0 + r2) / (xi_ + std::sqrt(1.0 + (1.0 - xi_ * xi_) * r2));
            pc_norm_plane /= pc_norm_plane.z();
        }

        virtual void worldToImage(const Eigen::Vector3d &pc, Eigen::Vector2d &pixel)  override{

        }

        //ceres优化用
        virtual void getParams(std::vector<double> &params) override{

        }

    };
}

#endif //RTSLAM_CAMERA_H
