#ifndef QUATERNION_H
#define	QUATERNION_H

class Quaternion
{
public:
    Quaternion(double x, double y, double z, double w)
    {
        this->x_ = x;
        this->y_ = y;
        this->z_ = z;
        this->w_ = w;
    }
    virtual ~Quaternion() {
    }

    void setW(double w) {
        this->w_ = w;
    }

    double w() const {
        return w_;
    }

    void setZ(double z) {
        this->z_ = z;
    }

    double z() const {
        return z_;
    }

    void setY(double y) {
        this->y_ = y;
    }

    double y() const {
        return y_;
    }

    void setX(double x) {
        this->x_ = x;
    }

    double x() const {
        return x_;
    }

    //Returns in radians
    static void toEulerAngle(const Quaternion& q, double& roll, double& pitch, double& yaw)
    {
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	yaw = atan2(siny, cosy);
    }

    static void toQuaternion( double yaw, double pitch, double roll, double& x, double& y,  double& z, double& w) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        w = cy * cp * cr + sy * sp * sr;
        x = cy * cp * sr - sy * sp * cr;
        y = sy * cp * sr + cy * sp * cr;
        z = sy * cp * cr - cy * sp * sr;
    }

    private:
        double x_;
        double y_;
        double z_;
        double w_;

};

#endif
