
import csv
import math
import svgpathtools
import numpy as np

# پارامترهای بازو
L1 = 0.6  # طول مفصل اول (متر)
L2 = 0.6  # طول مفصل دوم (متر)
steps_per_rad = 1000  # تعداد پالس در هر رادیان

# محاسبه معکوس (Inverse Kinematics)
def inverse_kinematics(x, y):
    r = math.hypot(x, y)
    if r > L1 + L2 or r < abs(L1 - L2):
        return None
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return theta1, theta2

# تبدیل SVG به CSV (پالس‌ها)
def process_svg(svg_file, output_csv, scale=0.001, step=2.0):
    paths, _ = svgpathtools.svg2paths(svg_file)
    points = []
    for path in paths:
        for seg in path:
            length = seg.length()
            num_points = max(2, int(length / step))
            for i in np.linspace(0, 1, num_points):
                pt = seg.point(i)
                points.append((pt.real * scale, pt.imag * scale))

    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        for (x, y) in points:
            angles = inverse_kinematics(x, y)
            if angles is None:
                continue
            theta1, theta2 = angles
            pulse1 = int(theta1 * steps_per_rad)
            pulse2 = int(theta2 * steps_per_rad)
            pen = 1
            writer.writerow([pulse1, pulse2, pen])

if __name__ == "__main__":
    process_svg("drawing.svg", "drawing.csv")
