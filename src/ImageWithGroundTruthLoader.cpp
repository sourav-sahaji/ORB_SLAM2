/*
* ImageWithGroundTruthLoader.cpp
*
*  Created on: 18 Feb 2016
*      Author: john
*/

//#include "stdafx.h"
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
//#include "rapidjson/schema.h"
#include "ImageWithGroundTruthLoader.h"


ImageWithGroundTruthLoader::ImageWithGroundTruthLoader(
    std::string filenamePrefix,
    std::string filenameSuffix,
    int imageCount,
    int initialIndex,
    int indexStep,
    int indexPad,
    const std::list<ImageFilterInterface*>& filters) :

    filenamePrefix(filenamePrefix),
    filenameSuffix(filenameSuffix),
    imageCount(imageCount),
    initialIndex(initialIndex),
    indexStep(indexStep),
    indexPad(indexPad),
    filters(filters)
{
}


ImageWithGroundTruthLoader::~ImageWithGroundTruthLoader()
{
}

int ImageWithGroundTruthLoader::getCount()
{
    return this->imageCount;
}

void ImageWithGroundTruthLoader::loadImages(std::vector<DatasetImage*>& destination, int indexOffset)
{
    for (int index = 0; index < imageCount; index++) {
        std::stringstream filenameBuilder;
        cv::Mat image;

        // Build the filename for the image
        int id = initialIndex + (indexStep * index);
        filenameBuilder << filenamePrefix;
        int tempPad = indexPad;
        while (id < tempPad && tempPad > 0) {
            filenameBuilder << '0';
            tempPad /= 10;
        }
        filenameBuilder << id;
        std::string imageFilename = filenameBuilder.str() + filenameSuffix;
        std::string groundTruthFilename = filenameBuilder.str() + "_ground_truth.txt";

        // Load the image, if we can.
        image = cv::imread(imageFilename.c_str(), cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cout << "Could not load image " << filenameBuilder.str() << std::endl;
            destination[index + indexOffset] = nullptr;
            continue;
        }

        // Load the ground truth from the file
        rapidjson::Document groundTruthJSON;
        char buffer[200];

        {
            bool valid = true;
            FILE *fp;
            if (fopen_s(&fp, groundTruthFilename.c_str(), "r") != 0) {
                std::cout << "Could not load ground truth " << filenameBuilder.str() << std::endl;
                valid = false;
            }

            rapidjson::FileReadStream jsonStream(fp, buffer, sizeof(buffer));
            groundTruthJSON.ParseStream(jsonStream);
            if (groundTruthJSON.HasParseError()) {
                std::cout << "Ground truth is invalid JSON: " << filenameBuilder.str() << std::endl;
                valid = false;
            }

            fclose(fp);

            if (!valid) {
                destination[index + indexOffset] = nullptr;
                continue;
            }
        }

        // Apply the filters
        if (filters.size() > 0) {
            ImageFilterInterface::applyFilters(image, filters);
        }

        // Read the ground truth from the JSON
        cv::Vec3d location(
            groundTruthJSON["location"]["x"].GetDouble(),
            groundTruthJSON["location"]["y"].GetDouble(),
            groundTruthJSON["location"]["z"].GetDouble());

        double rotW = groundTruthJSON["rotation"]["w"].GetDouble();
        double rotX = groundTruthJSON["rotation"]["x"].GetDouble();
        double rotY = groundTruthJSON["rotation"]["y"].GetDouble();
        double rotZ = groundTruthJSON["rotation"]["z"].GetDouble();

        // Create the DatasetImage object. It is the responsibility of the dataset to delete it later.
        destination[index + indexOffset] = new DatasetImage(image, location, this->quarternionToEuler(rotW, rotX, rotY, rotZ));
    }
}

/**
 * Wizardry to get the euler angles from the quarterion UE4 outputs.
 * See:
 * - Runtime\Core\Private\Math\UnrealMath.cpp
 * - http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * - http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
 */
cv::Vec3d ImageWithGroundTruthLoader::quarternionToEuler(double w, double x, double y, double z)
{
    double outX = 0, outY = 0, outZ = 0;

    const double singularityTest = z * x - w * y;
    const double yawY = 2.0 * (w * z + x * y);
    const double yawX = 1.0 - 2.0 * (y * y + z * z);

    // Original UE4 comment below:
    // reference
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

    // this value was found from experience, the above websites recommend different values
    // but that isn't the case for us, so I went through different testing, and finally found the case
    // where both of world lives happily.
    const double SINGULARITY_THRESHOLD = 0.4999995f;
    const double RAD_TO_DEG = (180.0) / CV_PI;

    if (singularityTest < -SINGULARITY_THRESHOLD)
    {
        outX = -90.f;
        outY = std::atan2(yawY, yawX) * RAD_TO_DEG;
        outZ = -outY - (2.f * std::atan2(x, w) * RAD_TO_DEG);
        outZ = std::fmod(outZ, 360.0);
        if (outZ > 180.0) {
            outZ -= 360.0;
        }
        else if (outZ < -180.0) {
            outZ += 360.0;
        }
    }
    else if (singularityTest > SINGULARITY_THRESHOLD)
    {
        outX = 90.f;
        outY = std::atan2(yawY, yawX) * RAD_TO_DEG;
        outZ = outY - (2.f * std::atan2(x, w) * RAD_TO_DEG);
        outZ = std::fmod(outZ, 360.0);
        if (outZ > 180.0) {
            outZ -= 360.0;
        } else if (outZ < -180.0) {
            outZ += 360.0;
        }
    }
    else
    {
        outX = std::asin(2.f*(singularityTest)) * RAD_TO_DEG;
        outY = std::atan2(yawY, yawX) * RAD_TO_DEG;
        outZ = std::atan2(-2.f*(w * x + y * z), (1.f - 2.f*(x * x + y * y))) * RAD_TO_DEG;
    }

    return cv::Vec3d(outX, outY, outZ);
}
