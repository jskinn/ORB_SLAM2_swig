/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2019 John Skinner <jskinn7 at gmail dot com>
* For more information see <https://github.com/jskinn/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VOCABULARY_BUILDER_H
#define VOCABULARY_BUILDER_H

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "ORBVocabulary.h"

namespace ORB_SLAM2
{

// A class to build ORB vocabularies, which will be loaded by the ORBSLAM system.
// Based on the DBoW demo example at https://github.com/dorian3d/DBoW2/blob/master/demo/demo.cpp, under a BSD license.
class VocabularyBuilder
{
public:
    VocabularyBuilder(
        int nfeatures = 500,
        float scaleFactor = (1.200000048F),
        int nlevels = 8,
        int edgeThreshold = 31,
        int firstLevel = 0,
        int WTA_K = 2,
        int scoreType = 0,
        int patchSize = 31,
        int fastThreshold = 20
    );

    void addImage(const cv::Mat& image);
    int getNumFeatures();
    void reset();
    void buildVocabulary(const std::string &strVocFile, int branchingFactor = 10, int numLevels = 6, int seed = 0) const;

private:
    cv::Ptr<cv::ORB> orb;
    std::vector<std::vector<cv::Mat>> features;
};

} //namespace ORB_SLAM

#endif // VOCABULARY_BUILDER_H
