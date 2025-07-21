#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>

#define EARTH_RADIUS 6371000.0

bool origin_set = false;
double origin_utm_x = 0.0;
double origin_utm_y = 0.0;

// =========================
// Konversi dan perhitungan 
// =========================

std::string convertUTCtoLocal(const std::string& timeUTC, int offsetHours) {
    if (timeUTC.length() < 6) return "";
    int hour = std::stoi(timeUTC.substr(0, 2));
    int minute = std::stoi(timeUTC.substr(2, 2));
    int second = std::stoi(timeUTC.substr(4, 2));
    hour += offsetHours;
    if (hour >= 24) hour -= 24;
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << hour << ":"
        << std::setw(2) << minute << ":"
        << std::setw(2) << second;
    return oss.str();
}

std::string convertToDecimalDegrees(const std::string& raw, const std::string& dir) {
    if (raw.empty() || raw.find('.') == std::string::npos || raw.length() < 4) return "";
    double deg = std::stod(raw.substr(0, raw.find('.') - 2));
    double min = std::stod(raw.substr(raw.find('.') - 2));
    double dec = deg + (min / 60.0);
    if (dir == "S" || dir == "W") dec *= -1;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << dec;
    return oss.str();
}

// =========================
// Konversi Ke UTM
// =========================

void latLonToUTM(double lat, double lon, double& utm_x, double& utm_y, int& zone, char& zone_letter) {
    const double a = 6378137.0;
    const double f = 1 / 298.257223563;
    const double k0 = 0.9996;
    const double e2 = f * (2 - f);
    const double n = f / (2 - f);
    const double e4 = e2 * e2;
    const double e6 = e4 * e2;

    zone = int((lon + 180) / 6) + 1;
    double lambda0 = (zone - 1) * 6 - 180 + 3;
    lambda0 *= M_PI / 180.0;

    double phi = lat * M_PI / 180.0;
    double lambda = lon * M_PI / 180.0;

    double N = a / sqrt(1 - e2 * sin(phi) * sin(phi));
    double T = tan(phi) * tan(phi);
    double C = e2 / (1 - e2) * cos(phi) * cos(phi);
    double A = cos(phi) * (lambda - lambda0);

    double M = a * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
        - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * sin(2 * phi)
        + (15 * e4 / 256 + 45 * e6 / 1024) * sin(4 * phi)
        - (35 * e6 / 3072) * sin(6 * phi));

    utm_x = k0 * N * (A + (1 - T + C) * pow(A, 3) / 6
        + (5 - 18 * T + T * T + 72 * C - 58 * e2 / (1 - e2)) * pow(A, 5) / 120) + 500000.0;

    utm_y = k0 * (M + N * tan(phi) * (A * A / 2
        + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24
        + (61 - 58 * T + T * T + 600 * C - 330 * e2 / (1 - e2)) * pow(A, 6) / 720));

    if (lat < 0)
        utm_y += 10000000.0;

    const char letters[] = "CDEFGHJKLMNPQRSTUVWX";
    int idx = int((lat + 80) / 8);
    zone_letter = (idx >= 0 && idx < 20) ? letters[idx] : 'Z';
}

// =========================
// Program Utama
// =========================

int main() {
    const char* port = "/dev/ttyUSB0";
    int serial_port = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port < 0) {
        std::cerr << "Gagal membuka port serial.\n";
        return 1;
    }

    termios tty{};
    tcgetattr(serial_port, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tcsetattr(serial_port, TCSANOW, &tty);

    char buf = '\0';
    std::string line;

    std::cout << std::fixed << std::setprecision(6);

    while (true) {
        int n = read(serial_port, &buf, 1);
        if (n > 0) {
            if (buf == '\n') {
                if (line.find("$GPRMC") == 0) {
                    std::stringstream ss(line);
                    std::string token, timeUTC, status, lat, latDir, lon, lonDir;
                    std::getline(ss, token, ','); // $GPRMC
                    std::getline(ss, timeUTC, ',');
                    std::getline(ss, status, ',');
                    std::getline(ss, lat, ',');
                    std::getline(ss, latDir, ',');
                    std::getline(ss, lon, ',');
                    std::getline(ss, lonDir, ',');

                    std::string timeLocal = convertUTCtoLocal(timeUTC, 7);
                    std::string lat_str = convertToDecimalDegrees(lat, latDir);
                    std::string lon_str = convertToDecimalDegrees(lon, lonDir);

                    if (!lat_str.empty() && !lon_str.empty()) {
                        double lat_d = std::stod(lat_str);
                        double lon_d = std::stod(lon_str);

                        double utm_x, utm_y;
                        int zone;
                        char zone_letter;
                        latLonToUTM(lat_d, lon_d, utm_x, utm_y, zone, zone_letter);

                        if (!origin_set) {
                            origin_utm_x = utm_x;
                            origin_utm_y = utm_y;
                            origin_set = true;
                        }

                        double rel_x = utm_x - origin_utm_x;
                        double rel_y = utm_y - origin_utm_y;

                        std::ofstream pos_file("position.txt");
                        if (pos_file.is_open()) {
                            pos_file << rel_x << " " << rel_y << std::endl;
                            pos_file.close();
                        }

                        std::cout << "Time       : " << timeLocal << "\n";
                        std::cout << "Position   : " << lat_d << ", " << lon_d << "\n";
                        std::cout << "UTM Zone   : " << zone << zone_letter << "\n";
                        std::cout << "UTM X (E)  : " << utm_x << " m\n";
                        std::cout << "UTM Y (N)  : " << utm_y << " m\n";
                        std::cout << "X          : " << rel_x << " m\n";
                        std::cout << "Y          : " << rel_y << " m\n";
                        std::cout << "----------------------------\n";
                    }
                }
                line.clear();
            } else if (buf != '\r') {
                line += buf;
            }
        }
    }

    close(serial_port);
    return 0;
}

