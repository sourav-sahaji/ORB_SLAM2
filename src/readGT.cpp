#include<stdio.h>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include<fstream>
#include<string>
#include<iomanip>

using namespace std;
using namespace cv;

Mat readDataFile(ifstream& inputFile, char delimiter);

/**
 * Wizardry to get the euler angles from the quarterion UE4 outputs.
 * See:
 * - Runtime\Core\Private\Math\UnrealMath.cpp
 * - http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * - http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
 */
cv::Vec3d quarternionToEuler(double w, double x, double y, double z)
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


int main()
{
    ifstream ccFile("gt-64.txt");
    if(!ccFile)
        cerr << "CC File not read" << endl;
    Mat inData = readDataFile(ccFile,' ');

    int inputCount = 0;
    ifstream folderList("/media/localuser/My Passport/slamDatasets/rendered/OrbSLAM datasets 2015-02-24/folderList2.txt");
    string ss;
    // For all the folders
    while(getline(folderList,ss))
    {
        cout << ss << endl;
        inputCount++;

//        if(inputCount != 1)
//            continue;
        stringstream outFileName;
        outFileName << "gt-" << inputCount << ".txt";
        ofstream gtFileOut;
        gtFileOut.open(outFileName.str().c_str());
        gtFileOut << fixed;

        // Add the noon-baseline data first
        for(int i1=0; i1<inData.rows; i1++)
        {
            gtFileOut << setprecision(8);

            for(int j1=0; j1<inData.cols; j1++)
                gtFileOut << inData.at<double>(i1,j1) << " ";

            gtFileOut << endl;
        }

        // For all the gt files
        int gtCounter=0;
        while(gtCounter<=549)
        {
            stringstream fileName;
            fileName << ss.c_str() << "Image_" << gtCounter << "_ground_truth.txt";

            // Load the ground truth from the file
            rapidjson::Document groundTruthJSON;
            char buffer[200];

            {
                //    bool valid = true;
                FILE *fp = fopen(fileName.str().c_str(), "r");
                if (fp==NULL) {
                    std::cout << "Could not load ground truth " << fileName.str() << std::endl;
                }

                rapidjson::FileReadStream jsonStream(fp, buffer, sizeof(buffer));
                groundTruthJSON.ParseStream(jsonStream);
                if (groundTruthJSON.HasParseError()) {
                    std::cout << "Ground truth is invalid JSON: " << fileName.str() << std::endl;
                }

                fclose(fp);
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

            cv::Vec3d rpy = quarternionToEuler(rotW, rotX, rotY, rotZ);

//            cout << rotZ /*<< " " << cv::Mat(location)*/ <<  endl;

            gtFileOut << setprecision(8) << gtCounter+550 << " " << location[0] << " " << location[1] << " " << location[2] << " "
                      << rotW << " " << rotX << " " << rotY << " " << rotZ << " " << rpy[0] << " " << rpy[1]
                      << " " << rpy[2] << endl;

            gtCounter++;
        }
    }
}


/*!
@brief Reads input data file into OpenCV matrix
@return Returns the Matrix
*/
Mat readDataFile(ifstream& inputFile, char delimiter)
{
    Mat dataMat;
    string buffer;
    while(getline(inputFile,buffer))
    {
        stringstream ss(buffer);
        string elem;
        Mat rowMat;
        while(getline(ss,elem,delimiter))
        {
            rowMat.push_back(atof(&elem[0]));
        }

        if(dataMat.empty())
        {
            dataMat.push_back(rowMat);
            dataMat = dataMat.t();
        }
        else
        {
            vconcat(dataMat,rowMat.t(),dataMat);
        }
    }
    return dataMat;
}
