# Copyright (c) 2014 - 2021 by Benjamin Wassermann
cmake_minimum_required(VERSION 2.6)

# Add source code properties


add_library(lib_lfd ${LIBRARY_TYPE}
../../include/lfd/FeatureFilter.hpp ../../include/lfd/FeatureDescriptor.hpp ../../include/lfd/FeatureMatcher.hpp ../../include/lfd/PairwiseLineMatcher.hpp 
../../include/lfd/FeatureDescriptorLBD.hpp ../../include/lfd/GlobalRotationFilter.hpp ../../include/lfd/StereoLineFilter.hpp
../../include/lfd/GenericDescriptor.hpp ../../include/lfd/LRDescriptor.hpp ../../include/lfd/StereoLineMatcher.hpp
../../include/lfd/MotionLineFilter.hpp ../../include/lfd/MotionLineMatcher.hpp ../../include/lfd/MotionDescriptor.hpp
)

SET_TARGET_PROPERTIES(lib_lfd PROPERTIES LINKER_LANGUAGE CXX)
