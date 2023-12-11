#pragma clang diagnostic push
#pragma ide diagnostic ignored "google-explicit-constructor"
#include <iostream>
#include <vector>
#include <sstream>
#include <array>
#include <map>
#include <algorithm>
#include <functional>

using namespace std;

class NSpace {
public:
    NSpace(u_int8_t dim);
    ~NSpace() = default;

    static u_int8_t getdim() {return m_dim;}

    class Point {
    public:
        Point(const vector<float>& crd, const vector<float>& tcb = {.0f, .0f, .0f});
        ~Point() = default;

        vector<float> getcrd() const {return m_crd;}
        vector<float> gettcb() const {return m_tcb;}
        void setcrd(const vector<float>& crd);
        void settcb(const vector<float>& tcb);

    private:
        vector<float> m_crd;
        vector<float> m_tcb;
    };

    static void addPoints(const vector<NSpace::Point> &points);
    static vector<Point> getPoints() {return m_points;}
    static void insertPoint(const NSpace::Point &point, uint index);
    static void removePoint(uint index);

    enum splineMode {
        basic, cyclic
    };

    static int fb(int _, int bound) { return (_ < 0) * _ ? _ != bound : bound - 1; }

    static int fc(int _, int bound) { return _ % bound; }

    map<splineMode, std::function<int(int, int)>> splineModeMap{{basic,  fb},
                                                                {cyclic, fc}};

    class Spline {
    public:
        Spline(splineMode mode = basic, uint size = 50) : m_mode(mode) {
            m_cfs.reserve(size);
        };
//        void insertcfs(vector);
    private:
        splineMode m_mode;
        vector<vector<array<float, 4>>> m_cfs;
    };

    Spline generateSpline(splineMode mode = basic, const vector<Point> &points = getPoints());

    friend ostream& operator<<(ostream& os, const Spline& spline);

    static array<float, 4> calculate(float P0,
                                     float P1, const vector<float> &TCB1,
                                     float P2, const vector<float> &TCB2,
                                     float P3);

private:
    static u_int8_t m_dim;
    static vector<NSpace::Point> m_points;
};

NSpace::NSpace(u_int8_t dim) {
    m_dim = dim;
}

NSpace::Point::Point(const vector<float>& crd, const vector<float>& tcb) : m_crd(crd), m_tcb(tcb) {}


void NSpace::Point::setcrd(const vector<float> &crd) {
    if (crd.size() == getdim()) {
        m_crd = crd;
    }
    else {
        cout << "tough luck bro";
    }
}

void NSpace::Point::settcb(const vector<float> &tcb) {
    if (tcb.size() == 3 && all_of(tcb.begin(), tcb.end(), [](float f){return abs(f)<=1;})) {
        m_tcb = tcb;
    }
    else {
        cout << "tough luck bro";
    }
}


void NSpace::addPoints(const vector<NSpace::Point> &points) {
    //invalidargument control
    m_points.insert(m_points.end(), points.begin(), points.end());
}

void NSpace::insertPoint(const NSpace::Point &point, uint index) {
    //invalidargument control
    m_points.insert(m_points.begin() + index, point);
}

void NSpace::removePoint(uint index) {
    m_points.erase(m_points.begin() + index);
}

NSpace::Spline NSpace::generateSpline(NSpace::splineMode mode, const vector<Point> &points) {
    uint size = points.size();
    auto res = Spline(mode,size);
    for (int d_iter = 0; d_iter != m_dim; ++d_iter) {
        for (int i = 0; i != size; ++i) {
            vector<int> v;
            for (int v_iter = i - 1; v_iter != i + 2; ++v_iter) {
                v.push_back(splineModeMap[mode](v_iter, size));
            }
            res.m_cfs[d_iter].push_back(
                    calculate(
                            points[v[0]].getcrd()[d_iter],
                            points[v[1]].getcrd()[d_iter],
                            points[v[1]].gettcb(),
                            points[v[2]].getcrd()[d_iter],
                            points[v[2]].gettcb(),
                            points[v[3]].getcrd()[d_iter]
                    )
            );
        }
    }
    return res;
}

array<float, 4> NSpace::calculate(float P0, float P1, const vector<float> &TCB1, float P2, const vector<float> &TCB2, float P3) {
    float Oi = (1 - TCB1[0]) * (1 + TCB1[1]) * (1 + TCB1[2]) / 2;
    float Oo = (1 - TCB1[0]) * (1 - TCB1[1]) * (1 - TCB1[2]) / 2;
    float Ii = (1 - TCB2[0]) * (1 - TCB2[1]) * (1 + TCB2[2]) / 2;
    float Io = (1 - TCB2[0]) * (1 + TCB2[1]) * (1 - TCB2[2]) / 2;


    float T1 = Oi * (P1 - P0) + Oo * (P2 - P1);
    float T2 = Ii * (P2 - P1) + Io * (P3 - P2);

    return {
            2 * P1 - 2 * P2 + T1 + T2,
            -3 * P1 + 3 * P2 - 2 * T1 + T2,
            T1,
            P1
    };
}

ostream &operator<<(ostream &os, const NSpace::Spline &spline) {
    // something
    return os;
}

int main() {
    //u_int8_t dim;
    //cout << "Введите размерность:" << endl;
    //cin >> dim;

    auto Q = NSpace(3);

    vector<NSpace::Point> testpoints = {
            NSpace::Point({0.0, 1.0, 4.0}), //tcb: .0, .0, .0
            NSpace::Point({1.1, 2.2, 3.3}, {0.5, 0.6, 0.7}),
            NSpace::Point({4.1, 5.2, -6.3}, {-0.5, -0.6, -0.7}),
            NSpace::Point({-2.1, -5.2, 7.3}, {0.5, -0.6, 0.7}),
            //NSpace::Point({1.0, 2.0, 3.0, 4.0}) invalid_argument
    };

    Q.addPoints(testpoints);

    NSpace::Spline S = Q.generateSpline();

    cout << S;

    return 0;
}


#pragma clang diagnostic pop