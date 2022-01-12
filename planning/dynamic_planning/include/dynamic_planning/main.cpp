#include "spline.h"
#include "matplotlibcpp.h"

int main()
{
    std::vector<double> s;
    std::vector<double> p;
    s.push_back(0.0);
    s.push_back(8.8);
    p.push_back(0.0);
    p.push_back(-1.0);

    tk::spline sp;
    sp.set_boundary(tk::spline::first_deriv, tan(0.1), tk::spline::first_deriv, 0, false);
    sp.set_points(s, p);

    std::vector<double> ss;
    std::vector<double> pp;

    for(double s = 0.0; s <= 8.8; s+=0.2)
    {
        ss.push_back(s);
        pp.push_back(sp(s));
    }
    matplotlibcpp::plot(ss, pp);
    matplotlibcpp::show();
    return 0;

}
