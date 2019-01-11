#ifndef _SLAM_DATA_MODEL_HPP_
#define _SLAM_DATA_MODEL_HPP_


#include <iostream>
#include <vector>
#include <array>
#include <slam/ceres_line_problem.hpp>
#include <geometry/stereo.hpp>
#include <slam/robotCameras.hpp>
#include <lfd/FeatureDescriptor.hpp>

#include <edge/nfa.hpp>

//remove:
#include <geometry/cameracv.hpp>


namespace lsfm{

    constexpr int OBS_FLAGS_NONE = 0;
    constexpr int OBS_FLAGS_TRACKING = 1;
    constexpr int OBS_FLAGS_STEREO = 2;

    constexpr int MODEL_FLAGS_NONE = 0;
    constexpr int MODEL_FLAGS_INVALID = 1;
//    constexpr int MODEL_FLAGS_STEREO = 2;

    //! Single observation of a geometry object in frame
    //! Stores the relations from frame to geometry indices
    struct Observation {
        enum GeometryType { POINT, LINE };

        Observation(int frame_idx = -1, int f = OBS_FLAGS_NONE, int geometry_idx = -1, GeometryType geometry_type = LINE) :
            frame(frame_idx), flags(f), geometry(geometry_idx), geometryType(geometry_type){}


        int frame, geometry; // also store index/d for geometry data vector
        GeometryType geometryType;  // getModelIDs assumes that the back() ist the newest observation
        int flags;
    };

    typedef std::vector<Observation> ObservationList;
    typedef std::vector<int> StereoFrameList;

    //! Represents a Line in the Model and its relations
    //! to the observed linesegments in the frames
    template<class L3d>
    struct ModelLine{
        typedef typename L3d::float_type float_type;

        ModelLine(const L3d& line_3d = L3d(), int f = MODEL_FLAGS_NONE, const ObservationList& observation_list = ObservationList()) :
            line(line_3d), observations(observation_list), flags(f){}

        bool valid() const{
            return !(flags & MODEL_FLAGS_INVALID);
        }
/*
        int stereoFramesSize() const{
            return (stereoFrames.size());
        }
*/
        const std::vector<int> stereoObs() const{
            std::vector<int> sObs;
            for(int i = 0; i < observations.size(); ++i){
                if(observations[i].flags && OBS_FLAGS_STEREO)
                    sObs.push_back(i);
            }
            return sObs;
        }

        L3d line;
//        StereoFrameList stereoFrames;
        ObservationList observations;
        int flags;
    };


    //! Represents a Point in the Model and its relations
    //! to the observed points in the frames
    template<class P3d>
    struct ModelPoint{

        ModelPoint(const P3d& point_3d = P3d(), const ObservationList& observation_list = ObservationList()) :
            point(point_3d), observations(observation_list){}

        P3d point;
        ObservationList observations;

    };

    //! Stores a section in a vector by keeping start and end position of section
    struct Section {

        Section(int start_idx = 0, int end_idx = 0) : beg(start_idx), end(end_idx) {
            if (beg > end)
                std::swap(beg,end);
        }

        inline int size() const {
            return end - beg;
        }

        int beg, end;
    };

    //! store detected line
    struct DetectedLine{

        DetectedLine(int mdlIdx = -1) : modelIndex(mdlIdx) {    }

        int modelIndex;
        // Space for more
    };

    //! store detected point
    struct DetectedPoint{

        DetectedPoint(int mdlIdx = -1) : modelIndex(mdlIdx) {    }

        int modelIndex;
        // Space for more
    };

    // -----------------------------------------------------------------

    template<class FT, int nCams, class LS, class L3d, class LineDescriptor = cv::Mat, class P3d = cv::Point3_<FT>, class KeyPoint = cv::KeyPoint, class PointDescriptor = lsfm::FdMat<float>, class LSDetector = lsfm::LsdCC<FT>, class ImMatMap = std::map<std::string, cv::Mat>>
    class SlamModel {

    public:

        typedef FT float_type;
        typedef LS line_segment_type;
        typedef L3d line_3d_type;
        typedef Pose<FT> pose_type;
        typedef ModelLine<L3d> model_line_type;
        typedef std::vector<Section> SectionList;
        typedef std::vector<line_segment_type> LineSegmentList;
        typedef std::vector<DetectedLine> DetectedLineList;
        typedef std::vector<model_line_type> ModelLineList;
        typedef std::vector<LineDescriptor> LineDescriptorList;
        typedef std::vector<pose_type> PoseList;

        typedef std::vector<KeyPoint> KeyPointList;
        typedef std::vector<PointDescriptor> PointDescriptorList;
        typedef std::vector<DetectedPoint> DetectedPointList;
        typedef ModelPoint<P3d> model_point_type;
        typedef std::vector<model_point_type> ModelPointList;

        typedef Vec4<FT> CayleyLine;
        typedef Vec3<FT> OptiPoint;
        //typedef std::map<std::string, cv::Mat> ImMatMap;

        struct ResidualData {
            int frame, segment, model;
        };

        typedef std::vector<ResidualData> ResidualVector;

    private:
        SectionList framesLineSections;
        LineSegmentList lineSegments;
        DetectedLineList detectedLines;
        ModelLineList modeledLines;
        LineDescriptorList lineDescriptors;
        PoseList robotPoses;
        std::vector<CayleyLine> cayleyStore;
        std::vector<OptiPoint> point3DStore;

        KeyPointList keyPoints;                 // all keyPoints, sections define which are from which frame
        DetectedPointList detectedPoints;       // as many as points, currently contains model index
        SectionList framesPointSections;        // one section per frame
        PointDescriptorList pointDescriptors;   // one mat per frame (using opencv surf)
        ModelPointList modeledPoints;

        CameraPluecker<FT> cameras[nCams];

        std::vector<cv::Mat> images;
        std::vector<ImMatMap> imageMatMap;
        std::vector<EdgeSegmentVector> segments; // vec vec -> frames of segments
        typedef typename LSDetector::PointVector PTV;
        std::vector<PTV> indexes; // vec vec -> frames of indexVectors, related to the segments

        // avoid realloction
        ResidualVector residuals;

    public:

        SlamModel(const Camera<FT> *cams) {
            for(int i = 0; i < nCams; i++){
                cameras[i] = cams[i];
            }
        }

        SlamModel(const std::vector<Camera<FT>> & cams) {
            for(int i = 0; i < nCams; i++){
                cameras[i] = cams[i];
            }
        }

        SlamModel(){ }

        //! Add new Frame(s if using multiple camera setup) with new robot pose (for example for first new frame from new pose)
        //! When adding new poses and frames, the order has to be kept consistent with the order in which the relative Cam Poses were added
        void addData(const LineSegmentList& lines, const DetectedLineList& modelId, const ModelLineList& newModels, const pose_type& newPose, const cv::Mat& image){
            images.push_back(image);
            robotPoses.push_back(newPose);
            addData(lines, modelId, newModels);
        }

        void addData(const LineSegmentList& lines, const DetectedLineList& modelId, const ModelLineList& newModels, const pose_type& newPose){
            robotPoses.push_back(newPose);
            addData(lines, modelId, newModels);
        }

        void addData(const LineSegmentList& lines, const DetectedLineList& modelId, const pose_type& newPose) {
            robotPoses.push_back(newPose);
            addData(lines, modelId);
        }

        void addData(const pose_type& newPose) {
            robotPoses.push_back(newPose);
        }

        void addData(const ImMatMap& imm) {
            imageMatMap.push_back(imm);
        }

        //! Add new Frame(s if using multiple camera setup) without new robot pose
        //! When adding new poses and frames, the order has to be kept consistent with the order in which the relative Cam Poses were added
        void addData(const LineSegmentList& lines, const DetectedLineList& modelId, const ModelLineList& newModels, const cv::Mat& image){
            addData(image);
            addData(lines, modelId, newModels);
        }

        //! Add new Frame
        void addData(const cv::Mat& image){
            images.push_back(image);        // expensive copying
        }

		//! Add new line segments + detected lines by tracking (have to be same size as lines segments!) and new models that are created
        void addData(const LineSegmentList& lines, const DetectedLineList& modelId, const ModelLineList& newModels){
			addData(newModels);
            addData(lines, modelId);
        }

		//! Add new models that are created
		void addData(const ModelLineList& newModels) {
			modeledLines.insert(modeledLines.end(), newModels.begin(), newModels.end());
        }

		//! Add new line segments + detected lines by tracking (have to be same size as lines segments!) and update observations
        //! For stereoFrames the order when adding frames must be correct.
        void addData(const LineSegmentList& lines, const DetectedLineList& modelId) {

            int frame = framesLineSections.size(), geometryStart = lineSegments.size();
            framesLineSections.push_back(Section(geometryStart, geometryStart + lines.size()));

            lineSegments.insert(lineSegments.end(), lines.begin(), lines.end());
			detectedLines.insert(detectedLines.end(), modelId.begin(), modelId.end());

            // update observations & nr stereo obs
            for (size_t i = 0; i != lines.size(); ++i) {
                int gidx = geometryStart + i;
                int midx = detectedLines[gidx].modelIndex;
                //std::cout << geometryStart + i << ":" << midx << std::endl;
                if (midx > -1){
/*
                    if(modeledLines[midx].observations.size() > 0 && getCycleNr(modeledLines[midx].observations.back().frame) == getCycleNr(frame)){
                        // this can actually only happen if more than two overlaping cameras exist
                        if(!(modeledLines[midx].stereoFrames.size() > 0 && modeledLines[midx].stereoFrames.back() == modeledLines[midx].observations.back().frame))
                            modeledLines[midx].stereoFrames.push_back(modeledLines[midx].observations.back().frame);
                        modeledLines[midx].stereoFrames.push_back(frame);
                    }
*/
                    modeledLines[midx].observations.push_back(Observation(frame, lsfm::OBS_FLAGS_TRACKING, gidx));

                }
            }
		}

        //! add new model to already existing lines -- stereo
        void addData(const int lineId1, const int frameLine1, const int lineId2, const int frameLine2, model_line_type model, int flags, Observation::GeometryType geoType = Observation::GeometryType::LINE){
            detectedLines[framesLineSections[frameLine1].beg + lineId1].modelIndex = modeledLines.size();
            detectedLines[framesLineSections[frameLine2].beg + lineId2].modelIndex = modeledLines.size();

            model.observations.push_back(Observation(frameLine1, flags, framesLineSections[frameLine1].beg + lineId1, geoType));
            model.observations.push_back(Observation(frameLine2, flags, framesLineSections[frameLine2].beg + lineId2, geoType));

            modeledLines.push_back(model);
        }

        //! Add new segment and index information of lines' pixel locations
        void addData(const EdgeSegmentVector & segs, const PTV& indxs) {
            segments.push_back(segs);
            indexes.push_back(indxs);
        }


        //! Add new keypoints, their modelIds and their descriptors
        void addData(const KeyPointList& keyPts, const DetectedPointList& detectedPts) {

            int frame = framesPointSections.size(), geometryStart = keyPoints.size();
            framesPointSections.push_back(Section(geometryStart, geometryStart + keyPts.size()));

            keyPoints.insert(keyPoints.end(), keyPts.begin(), keyPts.end());
            detectedPoints.insert(detectedPoints.end(), detectedPts.begin(), detectedPts.end());

            // update observations & nr stereo obs
            for (size_t i = 0; i != keyPts.size(); ++i) {
                int gidx = geometryStart + i;
                int midx = detectedPoints[gidx].modelIndex;

                if (midx > -1){
                    modeledPoints[midx].observations.push_back(Observation(frame, lsfm::OBS_FLAGS_TRACKING, gidx));
                }
            }
        }


        //! add new model to already existing Points -- stereo
        void addData(const int pointId1, const int framePoint1, const int pointId2, const int framePoint2, model_point_type model, int flags, Observation::GeometryType geoType = Observation::GeometryType::POINT){
            detectedPoints[framesPointSections[framePoint1].beg + pointId1].modelIndex = modeledPoints.size();
            detectedPoints[framesPointSections[framePoint2].beg + pointId2].modelIndex = modeledPoints.size();

            model.observations.push_back(Observation(framePoint1, flags, framesPointSections[framePoint1].beg + pointId1, geoType));
            model.observations.push_back(Observation(framePoint2, flags, framesPointSections[framePoint2].beg + pointId2, geoType));

            modeledPoints.push_back(model);
        }

        //! Add point descriptors
        void addDataDescriptor(const std::vector<PointDescriptor> & pD) {
            pointDescriptors.insert(std::end(pointDescriptors), std::begin(pD), std::end(pD));
            //pointDescriptors.push_back(pD);
        }

        //! Add line descriptors
        void addDataLineDescriptor(const std::vector<LineDescriptor> & lD){
            lineDescriptors.insert(std::end(lineDescriptors), std::begin(lD), std::end(lD));
        }
        //! Add line descriptor coming in one cv::Mat, for example LBD
        void addDataLineDescriptor(const cv::Mat & lD){
            for(int i = 0; i < lD.rows; ++i){
                lineDescriptors.push_back(lD.row(i));
            }
        }

        //! return relative camera configuration
        const CameraPluecker<FT> & getCameraConfig(int idx){
            return cameras[idx];
        }
        //! return number of cameras
        int getNumCams(){
            return nCams;
        }

        //! When adding new poses and frames, the order has to be kept consistent with the order in which the relative Cam Poses were added
        void setCameraPoses(RobotCameras<FT, nCams> rCams) {
            this->robotCams = rCams;
        }

        //! When adding new poses and frames, the order has to be kept consistent with the order in which the relative Cam Poses were added
        /*void setCameraPoses(Eigen::Matrix<FT, 3, 3> intrinsicsCofK[nCams]){
            std::copy(std::begin(intrinsicsCofK), std::end(intrinsicsCofK), std::begin(camCofK));
        }*/

        //! returns the frame number with respect to the camNr and cycle
        inline int getFrameNr(int cycleNr, int camNr) const {
            return nCams * cycleNr + camNr;
        }

        //! get camera Index corresponding to a frame number
        inline int getCamNr(int frameNr) const {
            return frameNr % nCams;
        }

        //! get cycle number corresponding to the frame number
        inline int getCycleNr(int frameNr) const {
            return frameNr / nCams;
        }

		int getFrameNum() const {
            //return framesLineSections.size();
            //return robotPoses.size() * nCams;
            return std::max(imageMatMap.size(), std::max(images.size(), framesLineSections.size()));     // one should be filled with the correct number of frames
		}

        const KeyPointList getPoints(int frameNr){
            return KeyPointList(keyPoints.begin() + framesPointSections[frameNr].beg, keyPoints.begin() + framesPointSections[frameNr].end);
        }

        const PointDescriptorList getPointDescriptors(int frameNr){
            return PointDescriptorList(pointDescriptors.begin() + framesPointSections[frameNr].beg, pointDescriptors.begin() + framesPointSections[frameNr].end);
        }

        const PointDescriptor getPointDescriptor(int descNr){
            return pointDescriptors[descNr];
        }

        const pose_type& getRobotPose(int frameNr) const {
            return robotPoses[getCycleNr(frameNr)];
        }

        const PoseList& getRobotPoses() const {
            return robotPoses;
        }

        const ResidualVector& getResdiuals() const {
            return residuals;
        }

        const LineDescriptorList getLineDescriptors(int frameNr){
            return LineDescriptorList(lineDescriptors.begin() + framesLineSections[frameNr].beg, lineDescriptors.begin() + framesLineSections[frameNr].end);
        }
        const cv::Mat getLineDescriptorsMat(int frameNr){
            LineDescriptorList descList(lineDescriptors.begin() + framesLineSections[frameNr].beg, lineDescriptors.begin() + framesLineSections[frameNr].end);
            cv::Mat descMat;
            for (int i = 0; i < descList.size(); ++i) {
                descMat.push_back(descList[i]);
            }
            return descMat;
        }
        const LineDescriptor getLineDescriptor(int descNr){
            return lineDescriptors[descNr];
        }

		//! returns list of all observed lines
        const LineSegmentList& getLineSegments() const {
			return lineSegments;
		}

		//! returns a vector of Linesegments from a certain Frame
        LineSegmentList getLineSegments(int frameNr) const {
            return LineSegmentList(lineSegments.begin() + framesLineSections[frameNr].beg, lineSegments.begin() + framesLineSections[frameNr].end);
		}

		//! returns list of all detected lines
		const DetectedLineList& getDetectedLines() const {
			return detectedLines;
		}

		//! returns a list of detected lines from a certain Frame
		DetectedLineList getDetectedLines(int frameNr) const {
            return DetectedLineList(detectedLines.begin() + framesLineSections[frameNr].beg, detectedLines.begin() + framesLineSections[frameNr].end);
		}

        //! returns a list of detected lines from a certain Frame
        DetectedPointList getDetectedPoints(int frameNr) const {
            return DetectedPointList(detectedPoints.begin() + framesPointSections[frameNr].beg, detectedPoints.begin() + framesPointSections[frameNr].end);
        }

        //! returns a list of detected lines from a certain Frame
        DetectedLine& getDetectedLine(int frameNr, int lineNr)  {
            return detectedLines[framesLineSections[frameNr].beg + lineNr];
        }

        //! returns the image of a certain Frame
        cv::Mat getImageFrame(int frameNr) const {
            if (images.size() > frameNr)
				return images[frameNr];
			return cv::Mat();
        }

        const ModelLineList& getModeledLines() const {
            return modeledLines;
        }

        const ModelPointList& getModeledPoints() const {
            return modeledPoints;
        }

        /**
         * @brief getModelLineSegment calculates the 3D Segment over all observations of the model keeping the max values
         * @param index
         * @return
         */
        LineSegment3<FT> getModelLineSegment(int index) const {
            ModelLine<L3d> l = modeledLines[index];

            FT dmin = std::numeric_limits<FT>::max();
            FT dmax = std::numeric_limits<FT>::lowest();
            for_each(l.observations.begin(), l.observations.end(), [&,this](const Observation& o) {
                const int currentCycle = this->getCycleNr(o.frame);
                Camera<FT> cam = this->cameras[this->getCamNr(o.frame)];

                //TODO: check if correct (compute real pose for cam) -> seems correct
                cam.concat(this->robotPoses.at(currentCycle));
                //cam.pose(this->robotPoses.at(currentCycle));

                const line_segment_type &ls = this->lineSegments[o.geometry];
                FT tmp = l.line.normalDistance(lineFromPixel(cam.focal(),cam.offset(),cam.rotM(),cam.origin(),ls.startPoint()));
                dmin = std::min(dmin,tmp);
                dmax = std::max(dmax,tmp);
                tmp = l.line.normalDistance(lineFromPixel(cam.focal(),cam.offset(),cam.rotM(),cam.origin(),ls.endPoint()));
                dmin = std::min(dmin,tmp);
                dmax = std::max(dmax,tmp);
            });

            return LineSegment3<FT>(l.line,dmin,dmax);
        }

        std::vector<LineSegment3<FT>> getModelLineSegments(std::vector<int> lineIDs) const {
            std::vector<LineSegment3<FT>> ret;
            for_each(lineIDs.begin(), lineIDs.end(), [this, &ret](const int &id) {
                ret.push_back(getModelLineSegment(id));
            });
            return ret;
        }
       
        std::vector<LineSegment3<FT>> getModelLineSegments() const {
            std::vector<LineSegment3<FT>> ret;
            for(int i = 0; i < modeledLines.size(); ++i){
                ret.push_back(getModelLineSegment(i));
            }
            return ret;
        }


        /**
         * @brief getModelLineSegment calculates the 3D Segment over all observations of the model keeping the median values
         * @param index
         * @return
         */
        LineSegment3<FT> getModelLineSegmentMedian(int index) const {
            ModelLine<L3d> l = modeledLines[index];

            FT dmin = std::numeric_limits<FT>::max();
            FT dmax = std::numeric_limits<FT>::lowest();
            std::vector<FT> minVals, maxVals;

            for_each(l.observations.begin(), l.observations.end(), [&,this](const Observation& o) {
                const int currentCycle = this->getCycleNr(o.frame);
                Camera<FT> cam = this->cameras[this->getCamNr(o.frame)];

                //TODO: check if correct (compute real pose for cam) -> seems correct
                cam.concat(this->robotPoses.at(currentCycle));
                //cam.pose(this->robotPoses.at(currentCycle));

                const line_segment_type &ls = this->lineSegments[o.geometry];
                FT tmp = l.line.normalDistance(lineFromPixel(cam.focal(),cam.offset(),cam.rotM(),cam.origin(),ls.startPoint()));
                FT tmp2 = l.line.normalDistance(lineFromPixel(cam.focal(),cam.offset(),cam.rotM(),cam.origin(),ls.endPoint()));
                if(tmp > tmp2){
                    maxVals.push_back(tmp);
                    minVals.push_back(tmp2);
                } else {
                    maxVals.push_back(tmp2);
                    minVals.push_back(tmp);
                }
            });

            std::sort(maxVals.begin(), maxVals.end());
            std::sort(minVals.begin(), minVals.end());

            dmin = minVals[std::floor(minVals.size()/2)];
            dmax = maxVals[std::floor(maxVals.size()/2)];

            return LineSegment3<FT>(l.line,dmin,dmax);
        }

        std::vector<LineSegment3<FT>> getModelLineSegmentsMedian(std::vector<int> lineIDs) const {
            std::vector<LineSegment3<FT>> ret;
            for_each(lineIDs.begin(), lineIDs.end(), [this, &ret](const int &id) {
                ret.push_back(getModelLineSegmentMedian(id));
            });
            return ret;
        }

        std::vector<LineSegment3<FT>> getModelLineSegmentsMedian() const {
            std::vector<LineSegment3<FT>> ret;
            for(int i = 0; i < modeledLines.size(); ++i){
                ret.push_back(getModelLineSegmentMedian(i));
            }
            return ret;
        }

        //! get Model IDs where the oldest observation is not older than parameter
        std::vector<int> getModelIDs (const int latestObservationCycle) const {
            int latestObsFrame = latestObservationCycle * nCams;
            std::vector<int> ret;
            for(int i = 0; i < modeledLines.size(); ++i){
                if(modeledLines[i].observations.back().frame >= latestObsFrame)
                    ret.push_back(i);
            }
            return ret;
        }

		std::vector<Line3<FT>> getModelLines() const {
			std::vector<Line3<FT>> ret;
			for_each(modeledLines.begin(), modeledLines.end(), [this, &ret](const model_line_type &l) {
				ret.push_back(l.line);
			});
			return ret;
		}

        std::vector<Line3<FT>> getModelLines(std::vector<int> lineIDs) const {
            std::vector<Line3<FT>> ret;
            for_each(lineIDs.begin(), lineIDs.end(), [this, &ret](const int &id) {
                ret.push_back(modeledLines[id].line);
            });
            return ret;
        }

        //! Bundle Adjustment on certain frames and certain models
        void bundleAdjustmentOnFrames(const std::vector<int>& frameIds, const std::vector<int>& modelIds) {

            if (frameIds.empty())
                return;

            //google::InitGoogleLogging();
            ceres::Problem problem;
            ResidualData residual;

            residuals.clear();
            residuals.reserve(framesLineSections[frameIds[0]].size() * frameIds.size() * 2);

            for(int i : modelIds){
                for(int j = 0; j < modeledLines[i].observations.size(); j++){
                    if(std::find(frameIds.begin(), frameIds.end(), modeledLines[i].observations[j].frame) != frameIds.end()
                            && getCycleNr(modeledLines[i].observations[j].frame) < robotPoses.size()){
                        residual.frame = modeledLines[i].observations[j].frame;
                        residual.model = i;
                        residual.segment = modeledLines[i].observations[j].geometry;
                        residuals.push_back(residual);
                    }
                }
            }

            cayleyStore.clear();    // TODO: speed up -> don't take all lines, only those which are optimized AND correctly write them back after adjustment
            cayleyStore.reserve(modeledLines.size());
            for_each(modeledLines.begin(),modeledLines.end(),[this](const model_line_type &model) {
                cayleyStore.push_back(model.line.cayley());
            });

            for_each(residuals.begin(), residuals.end(), [&](ResidualData& data) {

                ceres::CostFunction* cost_function =
                    CayleyMultiCamError<FT, nCams>::Create(lineSegments[data.segment], static_cast<Camera<FT>>(cameras[getCamNr(data.frame)]));

             //   ceres::LossFunction *huberLoss = new ceres::HuberLoss(1.0);
                ceres::LossFunction *SoftLOneLoss = new ceres::SoftLOneLoss(1.0);
             //   ceres::LossFunction *CauchyLoss = new ceres::CauchyLoss(1.0);
                //NULL , // squared loss

                problem.AddResidualBlock(cost_function,
                                         SoftLOneLoss, //
                                         robotPoses[getCycleNr(data.frame)].slamOrigin(),
                                         robotPoses[getCycleNr(data.frame)].slamRot(),
                                         &cayleyStore[data.model][0]);
            });

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_SCHUR;       // ceres::ITERATIVE_SCHUR; ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations =  300;
            options.function_tolerance = 0.01;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;

            // write the adjusted cayley lines back into the model
            for(size_t i = 0; i != cayleyStore.size(); ++i) {
                modeledLines[i].line = line_3d_type::lineFromCayley(cayleyStore[i]);
            }

        }

        //! Bundle Adjustment on certain frames
        void bundleAdjustmentOnFrames(const std::vector<int>& frameIds) {

            if (frameIds.empty())
                return;

            //google::InitGoogleLogging();
            ceres::Problem problem;
            ResidualData residual;

            residuals.clear();
            residuals.reserve(framesLineSections[frameIds[0]].size() * frameIds.size() * 2);

            for(int i : frameIds){
                residual.frame = i;
                for(int j = framesLineSections[i].beg; j < framesLineSections[i].end; j++){

                    int currentModelIndex = detectedLines[j].modelIndex;
                    if(currentModelIndex < 0)
                        continue;

                    residual.segment = j;
                    residual.model = currentModelIndex;
                    residuals.push_back(residual);
                }
            }

            cayleyStore.clear();
            cayleyStore.reserve(modeledLines.size());
            for_each(modeledLines.begin(),modeledLines.end(),[this](const model_line_type &model) {
                cayleyStore.push_back(model.line.cayley());
            });

            for_each(residuals.begin(), residuals.end(), [&](ResidualData& data) {

                ceres::CostFunction* cost_function =
                    CayleyMultiCamError<FT, nCams>::Create(lineSegments[data.segment], cameras[getCamNr(data.frame)]);

             //   ceres::LossFunction *huberLoss = new ceres::HuberLoss(1.0);
                ceres::LossFunction *SoftLOneLoss = new ceres::SoftLOneLoss(1.0);
             //   ceres::LossFunction *CauchyLoss = new ceres::CauchyLoss(1.0);
                //NULL , // squared loss

                problem.AddResidualBlock(cost_function,
                                         SoftLOneLoss,
                                         robotPoses[getCycleNr(data.frame)].slamOrigin(),
                                         robotPoses[getCycleNr(data.frame)].slamRot(),
                                         &cayleyStore[data.model][0]);
            });


            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_SCHUR;       // ceres::ITERATIVE_SCHUR; ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations =  300;
            options.function_tolerance = 0.01;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;

            // write the adjusted cayley lines back into the model
            for(size_t i = 0; i != cayleyStore.size(); ++i) {
                modeledLines[i].line = line_3d_type::lineFromCayley(cayleyStore[i]);
            }

        }

        //! Bundle Adjustment on certain frames with certain cycles(frames) in which the pose is not optimized
        //  beware that cycleNr != frameNr
        void bundleAdjustmentOnFramesFixedPoses(const std::vector<int>& frameIds, const std::vector<int>& fixedCycleIds) {

            if (frameIds.empty())
                return;

            //google::InitGoogleLogging();
            ceres::Problem problem;
            ResidualData residual;

            residuals.clear();
            residuals.reserve(framesLineSections[frameIds[0]].size() * frameIds.size() * 2);

            for(int i : frameIds){
                residual.frame = i;
                for(int j = framesLineSections[i].beg; j < framesLineSections[i].end; j++){

                    int currentModelIndex = detectedLines[j].modelIndex;
                    if(currentModelIndex < 0)
                        continue;

                    residual.segment = j;
                    residual.model = currentModelIndex;
                    residuals.push_back(residual);
                }
            }

            cayleyStore.clear();
            cayleyStore.reserve(modeledLines.size());
            for_each(modeledLines.begin(),modeledLines.end(),[this](const model_line_type &model) {
                cayleyStore.push_back(model.line.cayley());
            });

            for_each(residuals.begin(), residuals.end(), [&](ResidualData& data) {
                //ResidualData& data = residuals[i];

                //   ceres::LossFunction *ceresLoss = new ceres::HuberLoss(1.0);
                ceres::LossFunction *ceresLoss = new ceres::SoftLOneLoss(1.0);
                //   ceres::LossFunction *ceresLoss = new ceres::CauchyLoss(1.0);
                //   ceres::LossFunction *ceresLoss = NULL ; // squared loss

                // Only optimize line (and not also pose), if Cycle Number is in fixedCycleIds
                if(std::find(fixedCycleIds.begin(), fixedCycleIds.end(), getCycleNr(data.frame)) != fixedCycleIds.end()){

                    ceres::CostFunction* cost_function =
                        CayleyMultiCamError<FT, nCams>::CreateFixedPose(lineSegments[data.segment], cameras[getCamNr(data.frame)], robotPoses[getCycleNr(data.frame)]);

                    problem.AddResidualBlock(cost_function,
                                             ceresLoss,
                                             &cayleyStore[data.model][0]);

                } else {

                    ceres::CostFunction* cost_function =
                        CayleyMultiCamError<FT, nCams>::Create(lineSegments[data.segment], cameras[getCamNr(data.frame)]);

                    problem.AddResidualBlock(cost_function,
                                             ceresLoss,
                                             robotPoses[getCycleNr(data.frame)].slamOrigin(),
                                             robotPoses[getCycleNr(data.frame)].slamRot(),
                                             &cayleyStore[data.model][0]);
                }

            });

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_SCHUR;       // ceres::ITERATIVE_SCHUR; ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations =  3000;
            options.function_tolerance = 0.00001;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;

            // write the adjusted cayley lines back into the model
            for(size_t i = 0; i != cayleyStore.size(); ++i) {
                modeledLines[i].line = line_3d_type::lineFromCayley(cayleyStore[i]);
            }

        }

        //! Bundle Adjustment on certain frames with fixed lines
        //  beware that cycleNr != frameNr
        void bundleAdjustmentOnFramesFixedLines(const std::vector<int>& frameIds) {

            if (frameIds.empty())
                return;

            //google::InitGoogleLogging();
            ceres::Problem problem;
            ResidualData residual;

            residuals.clear();
            residuals.reserve(framesLineSections[frameIds[0]].size() * frameIds.size() * 3);    // rough approximation

            for(int i : frameIds){
                residual.frame = i;
                for(int j = framesLineSections[i].beg; j < framesLineSections[i].end; j++){

                    int currentModelIndex = detectedLines[j].modelIndex;
                    if(currentModelIndex < 0)
                        continue;

                    residual.segment = j;
                    residual.model = currentModelIndex;
                    residuals.push_back(residual);
                }
            }

            cayleyStore.clear();
            cayleyStore.reserve(modeledLines.size());
            for_each(modeledLines.begin(),modeledLines.end(),[this](const model_line_type &model) {
                cayleyStore.push_back(model.line.cayley());
            });

            for_each(residuals.begin(), residuals.end(), [&](ResidualData& data) {
                //ResidualData& data = residuals[i];

                //   ceres::LossFunction *ceresLoss = new ceres::HuberLoss(1.0);
                   ceres::LossFunction *ceresLoss = new ceres::SoftLOneLoss(1.0);
                //   ceres::LossFunction *ceresLoss = new ceres::CauchyLoss(1.0);
                //ceres::LossFunction *ceresLoss = NULL ; // squared loss

                // Only optimize pose
                ceres::CostFunction* cost_function =
                        CayleyMultiCamError<FT, nCams>::CreateFixedLine(lineSegments[data.segment], cameras[getCamNr(data.frame)], cayleyStore[data.model]);

                problem.AddResidualBlock(cost_function,
                                         ceresLoss,
                                         robotPoses[getCycleNr(data.frame)].slamOrigin(),
                        robotPoses[getCycleNr(data.frame)].slamRot());

            });

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_SCHUR;       // ceres::ITERATIVE_SCHUR; ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations =  3000;
            options.function_tolerance = 0.0001;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;

            // write the adjusted cayley lines back into the model
            /*
            for(size_t i = 0; i != cayleyStore.size(); ++i) {
                modeledLines[i].line = line_3d_type::lineFromCayley(cayleyStore[i]);
            }
            */
        }


        void bundleAdjustmentOnFramesPoints(const std::vector<int>& frameIds, const std::vector<int>& modelIds) {

            if (frameIds.empty())
                return;

            //google::InitGoogleLogging();

            ceres::Problem problem;
            ResidualData residual;

            residuals.clear();
            residuals.reserve(framesPointSections[frameIds[0]].size() * frameIds.size() * 2);

            for(int i : modelIds){
                for(int j = 0; j < modeledPoints[i].observations.size(); j++){
                    if(std::find(frameIds.begin(), frameIds.end(), modeledPoints[i].observations[j].frame) != frameIds.end()
                            && getCycleNr(modeledPoints[i].observations[j].frame) < robotPoses.size()){
                        residual.frame = modeledPoints[i].observations[j].frame;
                        residual.model = i;
                        residual.segment = modeledPoints[i].observations[j].geometry;
                        residuals.push_back(residual);
                    }
                }
            }

            point3DStore.clear();
            point3DStore.reserve(modeledPoints.size());
            for_each(modeledPoints.begin(),modeledPoints.end(),[this](const model_point_type &model) {
                point3DStore.push_back(OptiPoint(model.point[0], model.point[1], model.point[2]));
            });

            for_each(residuals.begin(), residuals.end(), [&](ResidualData& data) {

                ceres::CostFunction* cost_function =
                    PointMultiCamError<FT, nCams>::Create(lsfm::Vec2<FT> (keyPoints[data.segment].pt.x, keyPoints[data.segment].pt.y), static_cast<Camera<FT>>(cameras[getCamNr(data.frame)]));

             //   ceres::LossFunction *huberLoss = new ceres::HuberLoss(1.0);
                ceres::LossFunction *SoftLOneLoss = new ceres::SoftLOneLoss(1.0);
             //   ceres::LossFunction *CauchyLoss = new ceres::CauchyLoss(1.0);
             // NULL -> // squared loss

                problem.AddResidualBlock(cost_function,
                                         SoftLOneLoss, //
                                         robotPoses[getCycleNr(data.frame)].slamOrigin(),
                                         robotPoses[getCycleNr(data.frame)].slamRot(),
                                         &point3DStore[data.model][0]);
            });

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;       // ceres::ITERATIVE_SCHUR; ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations =  50;
            options.function_tolerance = 0.1;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;

            // write the adjusted cayley lines back into the model
            for(size_t i = 0; i != point3DStore.size(); ++i) {
                modeledPoints[i].point = P3d(point3DStore[i][0], point3DStore[i][1], point3DStore[i][2]);
            }
        }


        static void bundleAdjustmentFindModelToGtLineTransformation(lsfm::Pose<FT> &transformation, const std::vector<lsfm::Line3<FT>> &modelLines, const std::vector<lsfm::LineSegment3<FT>> &gtLines) {

            if(modelLines.size() != gtLines.size() || modelLines.size() == 0)
                return;

            //google::InitGoogleLogging();
            ceres::Problem problem;

            for(int i = 0; i < modelLines.size(); ++i){

                ceres::CostFunction* cost_function =
                    ModelLinesToGtTransformationError<FT>::Create(modelLines[i], gtLines[i]);

             //   ceres::LossFunction *Loss = new ceres::HuberLoss(1.0);
                ceres::LossFunction *Loss = new ceres::SoftLOneLoss(1.0);
             //   ceres::LossFunction *Loss = new ceres::CauchyLoss(1.0);
             //   ceres::LossFunction *Loss = NULL; // -> // squared loss

                problem.AddResidualBlock(cost_function,
                                         Loss, //
                                         transformation.slamOrigin(),
                                         transformation.slamRot());
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::ITERATIVE_SCHUR;//ceres::SPARSE_SCHUR;       // ceres::ITERATIVE_SCHUR; ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations =  500;
            options.function_tolerance = 0.00001;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << std::endl;
        }


        //! same as good lines to track but returns model IDs
        std::vector<int> goodModelsToTrack(int maxLines, int frameNum){
            DetectedLineList currentlyDetected = getDetectedLines(frameNum);
            LineSegmentList currentlLines = getLineSegments(frameNum);

            std::vector<int> goodModelIDs, goodLines = goodLinesToTrack(currentlyDetected, currentlLines, maxLines, frameNum);
            std::transform(std::begin(goodLines), std::end(goodLines), std::back_inserter(goodModelIDs), [&currentlyDetected](int d) { return currentlyDetected[d].modelIndex; } );
            return goodModelIDs;
        }

        std::vector<int> goodLinesToTrack(int maxLines, int frameNum){
            DetectedLineList currentlyDetected = getDetectedLines(frameNum);
            LineSegmentList currentlLines = getLineSegments(frameNum);
            return goodLinesToTrack(currentlyDetected, currentlLines, maxLines, frameNum);
        }

        std::vector<int> goodLinesToTrack(DetectedLineList currentlyDetected, LineSegmentList currentlLines, int maxLines, int frameNr){

            // Priority: Seen from multiple Cameras, seen more often, length (not currently)
//            DetectedLineList currentlyDetected = getDetectedLines(frameNum);
//            LineSegmentList currentlLines = getLineSegments(frameNum);

            struct GoodLine{
                GoodLine(int i, int mi, int nObs, int sd, FT meanReprojE, FT l, FT rd, FT nfaV) : id(i), modelID(mi), numObs(nObs), stereoFramesNum(sd), meanReprojError(meanReprojE), length(l), robotDistance(rd), nfaVal(nfaV) {}

                int id;
                int modelID;
                int numObs;
                int stereoFramesNum;
                FT meanReprojError;
                FT length;
                FT robotDistance;
                FT nfaVal;

                bool operator<(GoodLine b)
                        {
                            // lines closer than 0.5m might not be as reliable
                            if(robotDistance > 1.5 && b.robotDistance < 1.5)
                                return true;
                            else if(b.robotDistance > 1.5 && robotDistance < 1.5)
                                return false;

                            if(numObs > 3 && b.numObs > 3){
                                if(meanReprojError < b.meanReprojError)
                                //if(nfaVal > b.nfaVal)
                                    return true;
                                else
                                    return false;
                            }
                            if(numObs != b.numObs){
                                if(numObs > b.numObs)
                                    return true;
                                else
                                    return false;
                            }
                            if(stereoFramesNum != b.stereoFramesNum){
                                if(stereoFramesNum > b.stereoFramesNum)
                                    return true;
                                else
                                    return false;
                            }
                            if(length > b.length)
                                return true;

                            return false;
                        }
            };

            std::vector<FT> n;

            lsfm::Pose<FT> currentPose;
            //if(robotPoses.size() > 0) currentPose = robotPoses.back();
            if(robotPoses.size() > 0){
                currentPose = robotPoses[getCycleNr(frameNr)];

                NfaBinom<short, FT, typename LSDetector::point_type> nfab(8, 1.0 / 8);
                nfab.update(imageMatMap[frameNr]["gx"], imageMatMap[frameNr]["gy"]);
                nfab.evalLV(segments.at(frameNr), indexes.at(frameNr), getLineSegments(frameNr), n);

            }

            if(n.size() != currentlyDetected.size())        // TODO: remove
                std::cout << "ERROR with nfa usage!!!!!!" << std::endl;

            // add all
            std::vector<GoodLine> preGoodLines;
            preGoodLines.reserve(currentlyDetected.size());
            for(int i = 0; i < currentlyDetected.size(); ++i){
                if(currentlyDetected[i].modelIndex > -1 && modeledLines[currentlyDetected[i].modelIndex].valid()){
                    preGoodLines.push_back(GoodLine(i, currentlyDetected[i].modelIndex, modeledLines[currentlyDetected[i].modelIndex].observations.size(),
                                           modeledLines[currentlyDetected[i].modelIndex].stereoObs().size(),
                                           reprojectionErrorModel(currentlyDetected[i].modelIndex),
                                           currentlLines[i].length(),
                                           modeledLines[currentlyDetected[i].modelIndex].line.distance(currentPose.origin()),
                                           n[i]));
                }
            }

            std::sort(preGoodLines.begin(), preGoodLines.end());

            std::vector<int> goodLines;
            goodLines.reserve(maxLines);
            for(int i = 0; i < preGoodLines.size() && goodLines.size() < maxLines; ++i){
                goodLines.push_back(preGoodLines[i].id);
            }
            return goodLines;
        }

        //! Calculate ReprojectionError for a certain frame using CameraPluecker
        template<class CameraType>
        bool reprojectionErrorFrame(int frame, FT & err){

            const CameraPluecker<FT> & camera = cameras[getCamNr(frame)];
            const Pose<FT> & rPose = robotPoses[getCycleNr(frame)];

            std::vector<L3d> models;
            LineSegmentList segments;

            for(int j = framesLineSections[frame].beg; j < framesLineSections[frame].end; j++){

                int currentModelIndex = detectedLines[j].modelIndex;
                if(currentModelIndex < 0)
                    continue;

                models.push_back(modeledLines[currentModelIndex].line);
                segments.push_back(lineSegments[j]);
            }

            CameraType cam(camera);
            cam.concat(Pose<FT> (rPose.origin(),rPose.orientation()));

            return reprojectionError(cam, models, segments, err);
        }

        //! Calculate ReprojectionError for a given configuration using CameraPluecker and Cayley (Vectors)
        //  err must be initialized correctly
        template<class CameraType>
        inline static bool reprojectionError(const CameraType & camera, const std::vector<L3d> & lineModels3d, const LineSegmentList & lineSegments2d, FT & err){

            if(lineModels3d.size() != lineSegments2d.size()){
                std::cout << "Err: Size of 3d and 2d lines should be the same for reprojection." << std::endl;
                return false;
            }

            bool  valid = true;
            for(int i = 0; i < lineModels3d.size(); ++i){
                valid = valid && reprojectionError(camera, lineModels3d[i], lineSegments2d[i], err);
            }
            return valid;
        }

        //! Calculate ReprojectionError for a given configuration using CameraPluecker and Cayley
        //  err must be initialized correctly
        inline static bool reprojectionError(const CameraPluecker<FT> & camera, const L3d & lineModel3d, const LS & lineSegment2d, FT & err){

            const CayleyLine cayleyL = lineModel3d.cayley();

            // convert pointer to cayley object and compute projection to get 2d line from cayley
            Line<FT> projectedLine = CameraPluecker<FT>::projectCayleyCof(camera.camMCOF(),camera.origin(),camera.rotM(),cayleyL);

            // empty line -> we can not compute the residuals
            if (projectedLine.empty())
                return false;

            FT res0 = 0, res1 = 0;
            lineSegment2d.error(projectedLine, res0, res1);
            err += std::abs(res0);
            err += std::abs(res1);

            return true;
        }

        //! Calculate ReprojectionError for a given configuration using Camera2P
        //  err must be initialized correctly
        template<class CameraType>
        inline static bool reprojectionError(const CameraType & camera, const L3d & lineModel3d, const LS & lineSegment2d, FT & err, FT & errA, FT & errB){

            Line<FT> projectedLine = camera.project(lineModel3d);

            // empty line -> we can not compute the residuals
            if (projectedLine.empty())
                return false;

            errA = 0, errB = 0;
            lineSegment2d.error(projectedLine, errA, errB);
            err += std::abs(errA);
            err += std::abs(errB);

            return true;
        }
        template<class CameraType>
        inline static bool reprojectionError(const CameraType & camera, const L3d & lineModel3d, const LS & lineSegment2d, FT & err){
            FT a, b;
            return reprojectionError(camera, lineModel3d, lineSegment2d, err, a, b);
        }


        FT reprojectionErrorModel(const int modelID){
            FT var, maxE;
            std::vector<std::pair<int,FT>> obsError;
            return reprojectionErrorModel(modelID, var, maxE, obsError);
        }


        FT reprojectionErrorModel(const int modelID, FT &var, FT &maxE){
            std::vector<std::pair<int,FT>> obsError;
            return reprojectionErrorModel(modelID, var, maxE, obsError);
        }

        //! Reprojection Error of a certain model (and its observations) using Camera2P
        FT reprojectionErrorModel(const int modelID, FT &variance, FT &maxE, std::vector<std::pair<int,FT>> &obsError){
            FT error = 0;
            FT errorSq = 0;
            variance = 0;
            maxE = 0;
            int validObs = 0;
            int obsId = 0;

            // Using one pass variance, might have problems with accuracy
            for_each(modeledLines[modelID].observations.begin(), modeledLines[modelID].observations.end(), [&](Observation& obs){

                if(getCycleNr(obs.frame) < robotPoses.size()){  // there might not be a valid pose for the latest observations

                    Camera<FT> cam(cameras[getCamNr(obs.frame)]);
                    cam.concat(robotPoses[getCycleNr(obs.frame)]);
                    Camera2P<FT> camP(cam);
                    //Line<FT> projectedLine = camP.project(modeledLines[modelID].line);

                    //LS & testLine = lineSegments[obs.geometry]; //getLineSegments(obs.frame)[obs.geometry];
                    FT e = FT(0); // = testLine.error(projectedLine);
                    reprojectionError(camP, modeledLines[modelID].line, lineSegments[obs.geometry], e);
                    error += e;
                    errorSq += e*e;
                    maxE = std::max(maxE, e);
                    ++validObs;
    //                std::cout << " error: " << e << " ";
                    obsError.push_back(std::pair<int,FT>(obsId, e));
                }
                ++obsId;
            });

            variance = (errorSq - (error * error)/validObs) / validObs;
            return error / validObs;
        }

        FT reprojectionErrorModelPoint(const int modelID){
            FT var, maxE;
            std::vector<std::pair<int,FT>> obsError;
            return reprojectionErrorModelPoint(modelID, var, maxE, obsError);
        }

        FT reprojectionErrorModelPoint(const int modelID, FT &var, FT &maxE){
            std::vector<std::pair<int,FT>> obsError;
            return reprojectionErrorModelPoint(modelID, var, maxE, obsError);
        }

        FT reprojectionErrorModelPoint(const int modelID, FT &variance, FT &maxE, std::vector<std::pair<int,FT>> &obsError){
            FT error = 0;
            FT errorSq = 0;
            variance = 0;
            maxE = 0;
            int validObs = 0;
            int obsId = 0;

            // Using one pass variance, might have problems with accuracy
            for_each(modeledPoints[modelID].observations.begin(), modeledPoints[modelID].observations.end(), [&](Observation& obs){

                if(getCycleNr(obs.frame) < robotPoses.size()){  // there might not be a valid pose for the latest observations

                    Camera<FT> cam(cameras[getCamNr(obs.frame)]);
                    cam.concat(robotPoses[getCycleNr(obs.frame)]);
//                    Camera2P<FT> camP(cam);
//                    Line<FT> projectedModel = camP.project(modeledLines[modelID].line);

                    lsfm::Vec2<FT> projectedPoint;
                    if(Camera<FT>::project(cam.projM(), (modeledPoints[modelID].point), projectedPoint)){

                        KeyPoint obsPoint = keyPoints[obs.geometry]; //getLineSegments(obs.frame)[obs.geometry];
                        FT eSq = (obsPoint.pt.x-projectedPoint.x()) * (obsPoint.pt.x-projectedPoint.x()) + (obsPoint.pt.y-projectedPoint.y()) * (obsPoint.pt.y-projectedPoint.y());
                        FT e = std::sqrt(eSq);
                        error += e;
                        errorSq += eSq;
                        maxE = std::max(maxE, e);
                        ++validObs;
        //                std::cout << " error: " << e << " ";
                        obsError.push_back(std::pair<int,FT>(obsId, e));
                    }
                }
                ++obsId;
            });

            variance = (errorSq - (error * error)/validObs) / validObs;
            return error / validObs;
        }

        line_segment_type projectModel(int modelID, int camNr, const pose_type& pose) const {
            Camera<FT> cam(cameras[camNr]);
            cam.concat(pose);
            // Camera2P<FT> camP(cam);  // SPEED: maybe faster than Pluecker Projection
            CameraPluecker<FT> camP(cam);
            LineSegment<FT> projectedModel = camP.project(getModelLineSegment(modelID));
            return projectedModel;
        }

        /**
         * @brief outlierRemoveOnModel removes observations which have a high reprojection error compared to the other observations in the model
         * @param modelID
         * @return
         */
        bool outlierRemoveOnModel(int modelID){

            FT variance, maxError;
            std::vector<std::pair<int,FT>> obsError;

            FT meanError = reprojectionErrorModel(modelID, variance, maxError, obsError);

            for(auto obs : obsError){
                if(obs.second > (2 * std::sqrt(variance) + meanError)){

                    std::cout << "Rm obs from frame: " << modeledLines[modelID].observations[obs.first].frame << " model: " << modelID << " obsNr: " << obs.first << " avgE: " << meanError << " var: " << variance << " e: " << obs.second << std::endl;

                    detectedLines[(modeledLines[modelID].observations[obs.first]).geometry] = -1;
                    modeledLines[modelID].observations.erase(modeledLines[modelID].observations.begin() + obs.first);
                }
            }

        }

        /**
         * @brief outlierRemoveOnModelPoint removes observations which have a high reprojection error compared to the other observations in the model
         * @param modelID
         * @return
         */
        bool outlierRemoveOnModelPoint(int modelID){

            FT variance, maxError;
            std::vector<std::pair<int,FT>> obsError;

            FT meanError = reprojectionErrorModelPoint(modelID, variance, maxError, obsError);

            for(auto obs : obsError){
                if(obs.second > (3 * std::sqrt(variance) + meanError)){

                    detectedPoints[(modeledPoints[modelID].observations[obs.first]).geometry] = -1;
                    modeledPoints[modelID].observations.erase(modeledPoints[modelID].observations.begin() + obs.first);
                    std::cout << "removing observation from model: " << modelID << " obsNr: " << obs.first << " avgE: " << meanError << " var: " << variance << " e: " << obs.second << std::endl;
                }
            }

        }

        //! Intended for debugging use, to inspect each individual frame, outputs image, returns true if image is valid (size().width > 0)
        bool showFrameLineData(int & frameId, cv::Mat &frameImg){
            LineSegmentList lsList;
            DetectedLineList dlList;
            lsfm::Pose<FT> pose;
            lsfm::CameraPluecker<FT> cam;
            getFrameLineData(frameId, lsList, dlList, pose, cam, frameImg);
            if(frameImg.size().width > 0){
                std::vector<std::string> modelIds;
                std::transform(dlList.begin(), dlList.end(), std::back_inserter(modelIds), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });

                frameImg = lsfm::drawLines<FT>(frameImg, lsList, modelIds);
                LineSegmentList modelSegments;
                std::vector<std::string> modelSegmentIds;

                for(int i = 0; i < lsList.size(); ++i){
                    if(dlList[i].modelIndex < 0)
                        continue;

                    CayleyLine clLine = modeledLines[dlList[i].modelIndex].line.cayley();
                    Line<FT> projectedLine = CameraPluecker<FT>::projectCayleyCof(cam.camMCOF(),pose.origin(),pose.rotM(),*reinterpret_cast<const Vec4<FT>*>(&clLine[0]));

                    // Schnittpunkte auf projectedLine bestimmen
                    FT d = projectedLine.distance(lsList[i].startPoint());
                    typename line_segment_type::point_type ep, sp = projectedLine.normalLineDist(-d, lsList[i].startPoint());

                    d = projectedLine.distance(lsList[i].endPoint());
                    ep = projectedLine.normalLineDist(-d, lsList[i].endPoint());

                    // LineSegment plotten
                    modelSegments.push_back(line_segment_type(sp, ep));
                    modelSegmentIds.push_back("  " + std::to_string(dlList[i].modelIndex));

                }
                frameImg = lsfm::drawLines<FT>(frameImg, modelSegments, modelSegmentIds, cv::Scalar(255,255,0));

                /*  // Used to check if projections calculate the same result -> correct
                lsfm::CameraCV<FT>camProj(cam);
                camProj.pose(pose);
                std::vector<lsfm::LineSegment<FT>> models2d0;
                camProj.project(this->getModelLineSegments(),models2d0);
                frameImg = lsfm::drawLines<FT>(frameImg, models2d0);
                */
                return true;
            }
            return false;
        }

        //! Intended for debugging use, to inspect each individual frame, outputs image, returns true if image is valid (size().width > 0)
        bool showFrameLineDataCam2P(int & frameId, cv::Mat &frameImg){
            LineSegmentList lsList;
            DetectedLineList dlList;
            lsfm::Pose<FT> pose;
            Camera<FT> cam;
            getFrameLineData(frameId, lsList, dlList, pose, cam, frameImg);

            cam.concat(robotPoses[getCycleNr(frameId)]);
            Camera2P<FT> camP(cam);

            if(frameImg.size().width > 0){
                std::vector<std::string> modelIds;
                std::transform(dlList.begin(), dlList.end(), std::back_inserter(modelIds), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });

                frameImg = lsfm::drawLines<FT>(frameImg, lsList, modelIds);
                LineSegmentList modelSegments;
                std::vector<std::string> modelSegmentIds;

                for(int i = 0; i < lsList.size(); ++i){
                    if(dlList[i].modelIndex < 0)
                        continue;

                    //CayleyLine clLine = modeledLines[dlList[i].modelIndex].line.cayley();
                    //Line<FT> projectedLine = CameraPluecker<FT>::projectCayleyCof(cam.camMCOF(),pose.origin(),pose.rotM(),*reinterpret_cast<const Vec4<FT>*>(&clLine[0]));
                    Line<FT> projectedLine = camP.project(modeledLines[dlList[i].modelIndex].line);

                    // Schnittpunkte auf projectedLine bestimmen
                    FT d = projectedLine.distance(lsList[i].startPoint());
                    typename line_segment_type::point_type ep, sp = projectedLine.normalLineDist(d, lsList[i].startPoint());

                    d = projectedLine.distance(lsList[i].endPoint());
                    ep = projectedLine.normalLineDist(-d, lsList[i].endPoint());

                    // LineSegment plotten
                    modelSegments.push_back(line_segment_type(sp, ep));
                    modelSegmentIds.push_back("  " + std::to_string(dlList[i].modelIndex));

                }
                frameImg = lsfm::drawLines<FT>(frameImg, modelSegments, modelSegmentIds, cv::Scalar(255,255,0));

                /*  // Used to check if projections calculate the same result -> correct
                lsfm::CameraCV<FT>camProj(cam);
                camProj.pose(pose);
                std::vector<lsfm::LineSegment<FT>> models2d0;
                camProj.project(this->getModelLineSegments(),models2d0);
                frameImg = lsfm::drawLines<FT>(frameImg, models2d0);
                */
                return true;
            }
            return false;
        }

        //! Intended for debugging use, to inspect each individual frame
        template< class CamType >
        void getFrameLineData(int &frameId, LineSegmentList &lsList, DetectedLineList &dlList, Pose<FT> &pose, CamType &cam, cv::Mat &frameImg){
            if(framesLineSections.size() <= 0)
                return;
            if(frameId < 0)
                frameId = 0;
            if(frameId >= static_cast<int>(framesLineSections.size()))
                frameId = static_cast<int>(framesLineSections.size()) - 1;

            lsList = getLineSegments(frameId);
            dlList = getDetectedLines(frameId);

            cam = getCameraConfig(getCamNr(frameId));
            Pose<FT> tmpPose(cam);
            tmpPose.concat(getRobotPose(frameId));
            pose = tmpPose;

            if(images.size() > frameId)
                frameImg = images[frameId];

        }

        void transformAllPoses(const lsfm::Pose<FT> & transformation){
            for(int i = 0; i < robotPoses.size(); ++i){
                robotPoses[i].concat(transformation);
            }
        }

        void transformAllLines(const lsfm::Pose<FT> & transformation){
            for(int i = 0; i < modeledLines.size(); ++i){
                L3d l3 = modeledLines[i].line;

                auto v = l3.direction();
                auto po = l3.origin();
                auto pov = po + v;

                lsfm::Vec4<FT> p1h(po[0], po[1], po[2], 1);
                lsfm::Vec4<FT>p1ht = transformation.homM() * p1h;
                lsfm::Vec4<FT> p2h(pov[0], pov[1], pov[2], 1);
                lsfm::Vec4<FT>p2ht = transformation.homM() * p2h;
                lsfm::Vec3<FT> p1, p2;
                if(p1ht[3] > FT(0))
                    p1 = lsfm::Vec3<FT>(p1ht[0] / p1ht[3], p1ht[1] / p1ht[3], p1ht[2] / p1ht[3]);
                else
                    p1 = lsfm::Vec3<FT>(p1ht[0], p1ht[1], p1ht[2]);

                if(p2ht[3] > FT(0))
                    p2 = lsfm::Vec3<FT>(p2ht[0] / p2ht[3], p2ht[1] / p2ht[3], p2ht[2] / p2ht[3]);
                else
                    p2 = lsfm::Vec3<FT>(p2ht[0], p2ht[1], p2ht[2]);

                modeledLines[i].line = modeledLines[i].line = L3d(p1, p2 - p1);
            }
        }

    };

}


#endif
