
#ifndef __DLIB_TRANSFORM_HPP
#define __DLIB_TRANSFORM_HPP


#include <dlib/optimization.h>
#include <iostream>
#include <vector>
#include <geometry/line3.hpp>
#include <geometry/pose.hpp>


using namespace std;
using namespace dlib;

// ----------------------------------------------------------------------------------------

typedef matrix<double,2,1> input_vector;
typedef matrix<double,6,1> parameter_vector;

typedef std::vector<std::pair<std::vector<lsfm::Line3<FT>>, std::vector<lsfm::LineSegment3<FT>>>> dataVec;

// ----------------------------------------------------------------------------------------

// We will use this function to generate data.  It represents a function of 2 variables
// and 3 parameters.   The least squares procedure will be used to infer the values of
// the 3 parameters based on a set of input/output pairs.
double model (
    const input_vector& input,
    const parameter_vector& params
)
{
    const double p0 = params(0);
    const double p1 = params(1);
    const double p2 = params(2);

    const double i0 = input(0);
    const double i1 = input(1);

    const double temp = p0*i0 + p1*i1 + p2;

    return temp*temp;
}

// ----------------------------------------------------------------------------------------

// This function is the "residual" for a least squares problem.   It takes an input/output
// pair and compares it to the output of our model and returns the amount of error.  The idea
// is to find the set of parameters which makes the residual small on all the data pairs.
FT residual (
    const std::pair<lsfm::Line3<FT>,lsfm::LineSegment3<FT>> & data,
    const parameter_vector& params
)
{
    lsfm::Line3<FT> modelLine = data.first;
    lsfm::LineSegment3<FT> gtLine = data.second;

    FT r[3];
    r[0] = params(3);
    r[1] = params(4);
    r[2] = params(5);

    FT t[3];
    t[0] = params(0);
    t[1] = params(1);
    t[2] = params(2);
    lsfm::Pose<FT> tmpPose(t, r);

    lsfm::Vec4<FT> p1h(modelLine.origin()[0], modelLine.origin()[1], modelLine.origin()[2], 1);
    lsfm::Vec4<FT>p1ht = tmpPose.homM() * p1h;

    lsfm::Vec4<FT> p2h(modelLine.origin()[0] + modelLine.direction()[0], modelLine.origin()[1] + modelLine.direction()[1], modelLine.origin()[2] + modelLine.direction()[2], 1);
    lsfm::Vec4<FT>p2ht = tmpPose.homM() * p2h;

    lsfm::Vec3<FT> p1(p1ht[0] / p1ht[3], p1ht[1] / p1ht[3], p1ht[2] / p1ht[3]);
    lsfm::Vec3<FT> p2(p2ht[0] / p2ht[3], p2ht[1] / p2ht[3], p2ht[2] / p2ht[3]);

    lsfm::Line3<FT> transformedLine(p1, p2 - p1);

    //residuals[0] = transformedLine.distance(gtLine);

    FT r1 = std::abs( transformedLine.distance(gtLine.startPoint()) );
    FT r2 = std::abs( transformedLine.distance(gtLine.endPoint()) );

    FT rSum = r1 + r2;

    return rSum;//model(data.first, params) - data.second;
}

FT res (
    const dataVec& data,
    const parameter_vector& params
)
{
    return 1;
}

// ----------------------------------------------------------------------------------------

int dlibOptiModelToGtLineTransformation(lsfm::Pose<FT> &transformation, const std::vector<lsfm::Line3<FT>> &modelLines, const std::vector<lsfm::LineSegment3<FT>> &gtLines)
{
    try
    {
        dataVec lines;

        std::vector<std::pair<lsfm::Line3<FT>,lsfm::LineSegment3<FT>> > data_samples;
        for(int i = 0; i < modelLines.size(); ++i){
            std::pair<lsfm::Line3<FT>, lsfm::LineSegment3<FT>>(modelLines[i], gtLines[i]);
            data_samples.push_back(std::pair<lsfm::Line3<FT>, lsfm::LineSegment3<FT>>(modelLines[i], gtLines[i]));
        }

        parameter_vector x;
        x(0) = transformation.origin()[0];
        x(1) = transformation.origin()[1];
        x(2) = transformation.origin()[2];
        x(3) = transformation.orientation()[0];
        x(4) = transformation.orientation()[1];
        x(5) = transformation.orientation()[2];
/*
        // randomly pick a set of parameters to use in this example
        const parameter_vector params = 10*randm(3,1);
        cout << "params: " << trans(params) << endl;

        // Now let's generate a bunch of input/output pairs according to our model.
        input_vector input;
        for (int i = 0; i < 1000; ++i)
        {
            input = 10*randm(2,1);
            const double output = model(input, params);

            // save the pair
            data_samples.push_back(make_pair(input, output));
        }
*/
//        parameter_vector x;
        //x = 1;
        cout << "Use Levenberg-Marquardt, approximate derivatives" << endl;
        // If we didn't create the residual_derivative function then we could
        // have used this method which numerically approximates the derivatives for you.
        solve_least_squares_lm(objective_delta_stop_strategy(1e-7).be_verbose(),
                               residual,
                               derivative(residual),
                               data_samples,
                               x);

        // Now x contains the solution.  If everything worked it will be equal to params.
        cout << "inferred parameters: "<< trans(x) << endl;
        //cout << "solution error:      "<< length(x - params) << endl;
        cout << endl;

        transformation.origin(lsfm::Vec3<FT>(x(0), x(1), x(2)));
        transformation.orientation(lsfm::Vec3<FT>(x(3), x(4), x(5)));
    }
    catch (std::exception& e)
    {
        cout << e.what() << endl;
    }
}





















#endif
