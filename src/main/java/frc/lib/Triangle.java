package frc.lib;

public class Triangle {
    double m_sidea;
    double m_sideb;
    double m_sidec;

    public Triangle(double side_a, double side_b, double side_c) {
        m_sidea = side_a;
        m_sideb = side_b;
        m_sidec = side_c;
    }

    public double getAngleA() {
        return Math.acos((Math.pow(m_sideb, 2) - Math.pow(m_sidea, 2) - Math.pow(m_sidec, 2)) / (-2 * m_sidea * m_sidec));
    }

    public double getAngleB() {
        return Math.acos((Math.pow(m_sidec, 2) - Math.pow(m_sidea, 2) - Math.pow(m_sideb, 2)) / (-2 * m_sidea * m_sideb));
    }

    public double getAngleC() {
        return Math.acos((Math.pow(m_sidea, 2) - Math.pow(m_sideb, 2) - Math.pow(m_sidec, 2)) / (-2 * m_sideb * m_sidec));
    }
}