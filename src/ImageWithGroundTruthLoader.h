/*
* ImageWithGroundTruthLoader.h
*
*  Created on: 18 Feb 2016
*      Author: john
*/

#ifndef IMAGEWITHGROUNDTRUTHLOADER_H_
#define IMAGEWITHGROUNDTRUTHLOADER_H_

#include <string>
#include <list>
#include "ImageFilterInterface.h"
#include "ImageLoaderInterface.h"

class ImageWithGroundTruthLoader :
	public ImageLoaderInterface
{
public:
	ImageWithGroundTruthLoader(
		std::string filenamePrefix,
		std::string filenameSuffix,
		int imageCount,
		int initialIndex,
		int indexStep,
		int indexPad,
		const std::list<ImageFilterInterface*>& filters = std::list<ImageFilterInterface*>());


	~ImageWithGroundTruthLoader();

	virtual int getCount();

	virtual void loadImages(std::vector<DatasetImage*>& destination, int indexOffset);

private:
	std::string filenamePrefix;
	std::string filenameSuffix;
	int imageCount;
	int initialIndex;
	int indexStep;
	int indexPad;
	const std::list<ImageFilterInterface*>& filters;

	cv::Vec3d quarternionToEuler(double w, double x, double y, double z);
};

#endif /* IMAGEWITHGROUNDTRUTHLOADER_H_ */
