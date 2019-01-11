#include <iostream>
#include <slam/line_jet.hpp>
#include <geometry/stereo.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>


using namespace std;
using namespace lsfm;

//std::ostream& operator<<(std::ostream &os, const ceres::Jet<double,12> &data) {
//    os << data.a;
//}

int main(int argc, char** argv)
{
    typedef ceres::Jet<double,12> MyJet;

    MyJet pe1[3] = {MyJet(-1.9983169059136842715673765269457362592220306396484375), MyJet(0.54713356553423242445433061220683157444000244140625), MyJet(2.524045441895548691491057979874312877655029296875)}, de1[3] = {MyJet(-3.0227787728442851999943741247989237308502197265625), MyJet(0.814020739838907214647178989253006875514984130859375), MyJet(3.41091596507640471003242055303417146205902099609375)};
    MyJet me[3], le[3];

    Vec3d p1(-1.9983169059136842715673765269457362592220306396484375, 0.54713356553423242445433061220683157444000244140625, 2.524045441895548691491057979874312877655029296875), d1(-3.0227787728442851999943741247989237308502197265625, 0.814020739838907214647178989253006875514984130859375, 3.41091596507640471003242055303417146205902099609375);
    Vec3d p2(4,6,7), d2(3,4,9);
    Vec3d p3(4,5,6), d3(1,3,8);
    LineSegment3<double> line1(p1,d1);
    Line3d line2(p2,d2);
    Line3d line3(p3,d3);
    Line3d linea = line3;

    Vec2d p2d1(38.89010411189400429066154174506664276123046875, 274.52544957408775871954276226460933685302734375), d2d1(58.02197917762121193163693533279001712799072265625, 270.0949100851825050995103083550930023193359375);
    Vec2d p2d2(4,6), d2d2(3,4);
    LineSegment2d line2d1(p2d1,d2d1);
    Line2d line2d2(p2d2,d2d2);


    Matx33<double> K = Matx33<double>::Zero();
    //K >> "332.3852944400267, 0, 333.452392578125; 0, 332.3852944400267, 195.2011489868164; 0, 0, 1";
    K(0,0) = 332.385294440026655138353817164897918701171875;
    K(0,2) = 333.452392578125;
    K(1,1) = 332.385294440026655138353817164897918701171875;
    K(1,2) = 195.20114898681640625;
    K(2,2) = 1.0;

    Matx33<double> R = Matx33<double>::Identity();

    Vec3d c(0,0,0);
    //std::cout.precision(500);
    Vec3d m = linea.momentum(),l = linea.direction(), sp, nm, nl;
    std::cout << "Usual way: " << std::endl;
    std::cout << " m: " << m << ", l: " << l << std::endl;

    double wp;
    linea.cayleyRepresentationFromPluecker(m,l, wp, sp);
    std::cout << " wp: " << wp << ", sp: " << sp << std::endl;

    linea.plueckerCoordinatesFromCayley(wp, sp, nm, nl);
    std::cout << " nm: " << nm << ", nl: " << nl << std::endl;

    Line<double> testLine2dUsual = CameraPlueckerd::projectPlueckerM(K,c,R,nm, nl);

    //double line2d2Arr[3] = {line2d2.x;
    //std::cout << " error1: " << line2d1.error(line2d2) << std::endl;
    std::cout << " error2: " << line2d1.error(testLine2dUsual) << std::endl;

    //std::cout << "line3d: " << linea.startPoint() << " endpoint: " << linea.endPoint() << std::endl;
    std::cout << "aline3d direction: " << linea.direction() << " momentum:" << linea.momentum() << " origin: " << linea.origin() << std::endl;

    Line3d nline = Line3d::lineFromPluecker(nm,nl);

    std::cout << "nline3d direction: " << nline.direction() << " momentum:" << nline.momentum() << " origin: " << nline.origin() << std::endl;

    std::cout << "distance: " << line2.distance(line3)  << std::endl;


    std::cout << std::endl << "Jet way: " << std::endl;

    plueckerCoordinates(pe1, de1, me, le);
    std::cout << " me: " << me[0] << ", le: " << le[0] << std::endl;
    std::cout << " me: " << me[1] << ", le: " << le[1] << std::endl;
    std::cout << " me: " << me[2] << ", le: " << le[2] << std::endl;

    MyJet w, s[3], mp[3], lp[3];
    cayleyRepresentationFromPluecker<MyJet>(me,le,w,s);

    std::cout << " s: " << s[0] << ", w: " << w << std::endl;
    std::cout << " s: " << s[1] << std::endl;
    std::cout << " s: " << s[2] << std::endl;

    plueckerCoordinatesFromCayley(w, &s[0], mp, lp);
    std::cout << " nm: " << mp[0] << ", nl: " << lp[0] << std::endl;
    std::cout << " nm: " << mp[1] << ", nl: " << lp[1] << std::endl;
    std::cout << " nm: " << mp[2] << ", nl: " << lp[2] << std::endl;

    Eigen::Matrix<MyJet, 3, 3> Kp, Rp;
    Kp(0,0) = MyJet(332.385294440026655138353817164897918701171875);
    Kp(0,2) = MyJet(333.452392578125);
    Kp(1,1) = MyJet(332.385294440026655138353817164897918701171875);
    Kp(1,2) = MyJet(195.20114898681640625);
    Kp(2,2) = MyJet(1.0);
    Kp(0,1) = MyJet(0.0);
    Kp(1,0) = MyJet(0.0);
    Kp(2,0) = MyJet(0.0);
    Kp(2,1) = MyJet(0.0);

    Rp(0,0) = MyJet(1.0);
    Rp(0,1) = MyJet(0.0);
    Rp(0,2) = MyJet(0.0);
    Rp(1,0) = MyJet(0.0);
    Rp(1,1) = MyJet(1.0);
    Rp(1,2) = MyJet(0.0);
    Rp(2,0) = MyJet(0.0);
    Rp(2,1) = MyJet(0.0);
    Rp(2,2) = MyJet(1.0);

    MyJet cp[3];
    cp[0] = MyJet(0.0);
    cp[1] = MyJet(0.0);
    cp[2] = MyJet(0.0);

    MyJet testLine2Pluecker[3];
    projectPluecker<MyJet>(Kp,Rp,cp,mp, lp, testLine2Pluecker);
    //testLine2Pluecker = line1.projectPlueckerT<MyJet>(Kp,Rp,cp,mp, lp);

    MyJet ps[3], pe[3];
    ps[0] = MyJet(p2d1.x());
    ps[1] = MyJet(p2d1.y());
    ps[2] = MyJet(1.0);
    pe[0] = MyJet(d2d1.x());
    pe[1] = MyJet(d2d1.y());
    pe[2] = MyJet(1.0);

    std::cout << " errP " << errorSED(testLine2Pluecker, ps, pe) << std::endl;


    return 0;
}
