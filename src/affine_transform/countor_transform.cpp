#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include <iostream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
typedef boost::geometry::model::polygon<BoostPoint> BoostPolygon;

void warp_point(cv::Point &point, const cv::Mat &homography) {
  cv::Point tmp = point;

  double d = homography.at<double>(2, 0) * tmp.x +
            homography.at<double>(2, 1) * tmp.y + homography.at<double>(2, 2);

  // Check if d is zero or NaN
  if (d != 0 && !std::isnan(d)) {
    // clang-format off
    point.x = static_cast<int>((homography.at<double>(0, 0) * tmp.x +
                                homography.at<double>(0, 1) * tmp.y +
                                homography.at<double>(0, 2)) / d);
    point.y = static_cast<int>((homography.at<double>(1, 0) * tmp.x +
                                homography.at<double>(1, 1) * tmp.y +
                                homography.at<double>(1, 2)) / d);
    // clang-format on
  } else {
    std::cout << "NAN " << d << std::endl;
  }
}

BoostPolygon convertToBoostPolygon(const std::vector<cv::Point>& cvPolygon) {
    BoostPolygon boostPolygon;
    for (const auto& point : cvPolygon) {
        boost::geometry::append(boostPolygon, BoostPoint(point.x, point.y));
    }
    // Close the polygon by adding the first point at the end
    boost::geometry::correct(boostPolygon);
    return boostPolygon;
}

std::vector<BoostPolygon> computeUnion(const BoostPolygon& poly1, const BoostPolygon& poly2) {
    std::vector<BoostPolygon> output;
    boost::geometry::union_(poly1, poly2, output);
    return output;
}

std::vector<cv::Point> convertToCvPolygon(const BoostPolygon& boostPolygon) {
    std::vector<cv::Point> cvPolygon;
    for (const auto& point : boostPolygon.outer()) {
        cvPolygon.emplace_back(static_cast<int>(point.x()), static_cast<int>(point.y()));
    }
    return cvPolygon;
}


int main() {
    // Create a 200x200 white image
    cv::Mat image = cv::Mat::ones(200, 200, CV_8UC3) * 255;


    std::vector<cv::Point2i> countor {{1, 1} , {200, 1}, {200, 200}, {1, 200}};
    std::vector<cv::Point2i> countor2 {{50, 50} , {300, 50}, {300, 300}, {50, 300}};


    

    // Define a homography matrix (identity matrix for no transformation)
    cv::Mat homography_matrix = (cv::Mat_<double>(3, 3) << 
                    0.762959361076355,
                    -0.2090536504983902,
                    333.5301818847656,
                
                    0.022531522437930107,
                    0.6184757947921753,
                    2.4000786424949183e-07,
                
                    -5.409866389527451e-06,
                    -0.00023735851573292166,
                    1.0052505731582642
                        );

    // Apply the homography transformation to the image
    cv::Mat transformed_image;
    cv::warpPerspective(image, transformed_image, homography_matrix, cv::Size(800,800));
    std::vector<cv::Point2i> new_countor(countor);
    std::vector<cv::Point2i> new_countor2(countor2);
    
    
    for(std::size_t i = 0 ; i < new_countor.size() ; ++i) {
        warp_point(new_countor[i], homography_matrix);
    }
    for(std::size_t i = 0 ; i < new_countor2.size() ; ++i) {
        warp_point(new_countor2[i], homography_matrix);
    
    }
    BoostPolygon boostPolygon1 = convertToBoostPolygon(new_countor);
    BoostPolygon boostPolygon2 = convertToBoostPolygon(new_countor2);

    BoostPolygon boostPolygon3 = convertToBoostPolygon(countor);
    BoostPolygon boostPolygon4 = convertToBoostPolygon(countor2);


    std::vector<BoostPolygon> unionPolygons = computeUnion(boostPolygon1, boostPolygon2);


    std::vector<cv::Point> cvPolygon = convertToCvPolygon(unionPolygons[0]);


    std::cout << "after transorm " << std::endl;

    cv::polylines(transformed_image, countor, true, cv::Scalar(80, 80, 0), 3);
//    cv::polylines(transformed_image, new_countor, true, cv::Scalar(180, 180, 0), 3);
//    cv::polylines(transformed_image, countor2, true, cv::Scalar(0, 80, 80), 3);
//    cv::polylines(transformed_image, new_countor2, true, cv::Scalar(0, 180, 180), 3);

    cv::polylines(transformed_image, cvPolygon, true, cv::Scalar(0, 200, 200), 3);

    // Display the result on the screen
    cv::imshow("Transformed Image", transformed_image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
