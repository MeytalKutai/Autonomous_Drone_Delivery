import time
import cv2
import numpy as np
import math
import re
from OpenDJI import OpenDJI, EventListener

# כתובת IP של הרחפן
DRONE_IP = "192.168.215.55"

# טווח מת - מונע רעידות קטנות בגובה
DEADZONE = 0.1

# נקודת היעד (דוגמה - יש להחליף בנתונים אמיתיים)
TARGET_LAT = 31.9547160  # קו רוחב של היעד
TARGET_LON = 34.7895753  # קו אורך של היעד

# סף עצירה במרחק מהיעד (ביחידות GPS, כ-2 מטר)
STOP_DISTANCE_THRESHOLD = 0.00002  # כ-2 מטר

# תבנית לניתוח נתוני מיקום
NUM_REG = r'[-+]?\d+\.?\d*'
LOCATION_PATTERN = re.compile(
    r'{"latitude":(' + NUM_REG + r'),"longitude":(' + NUM_REG + r'),"altitude":(' + NUM_REG + r')}')

# משתנים גלובליים
current_data = {
    "altitude": 0.0,
    "latitude": 0.0,
    "longitude": 0.0,
    "yaw": 0.0
}

# משתנה לממוצע נע של הגובה
filtered_altitude = 0.0

# בקר PID מתוקן
class PIDController:
    def __init__(self, Kp, Ki, Kd, target):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.integral = 0
        self.previous_error = 0
        self.previous_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.previous_time
        if dt <= 0:
            return 0

        error = self.target - current_value
        if abs(error) < DEADZONE:
            return 0

        self.integral += error * dt
        self.integral = max(min(self.integral, 1.0), -1.0)

        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.previous_error = error
        self.previous_time = current_time
        # הפחתת המהירות האנכית המרבית
        return max(min(output, 0.1), -0.1)  # הפחתה מ-0.15 ל-0.1

# מאזינים לנתונים
class LocationListener(EventListener):
    def onValue(self, value):
        global current_data, filtered_altitude
        if value == "null":
            print("⚠️ נתוני מיקום חסרים")
            return
        try:
            match = LOCATION_PATTERN.fullmatch(value)
            if match:
                current_data["latitude"] = float(match.group(1))
                current_data["longitude"] = float(match.group(2))
                altitude = float(match.group(3))
                # מסנן ממוצע נע לגובה (שיפור להחלקה טובה יותר)
                filtered_altitude = 0.98 * filtered_altitude + 0.02 * altitude
                current_data["altitude"] = filtered_altitude
        except ValueError:
            print(f"⚠️ שגיאה בניתוח מיקום: {value}")

class YawListener(EventListener):
    def onValue(self, value):
        global current_data
        if value == "null":
            print("⚠️ נתוני Yaw חסרים")
            return
        try:
            current_data["yaw"] = float(value)
        except ValueError:
            print(f"⚠️ שגיאה בניתוח Yaw: {value}")

# פונקציה לחישוב הזווית ליעד
def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    bearing = math.degrees(math.atan2(x, y))
    bearing = (bearing + 360) % 360
    return bearing

# פונקציה להתחברות והמראה
def connect_and_takeoff():
    try:
        drone = OpenDJI(DRONE_IP)
        drone.__enter__()
        print("🔗 מחובר לרחפן בהצלחה")
        drone.enableControl(True)
        time.sleep(1)

        drone.listen(OpenDJI.MODULE_FLIGHTCONTROLLER, "AircraftLocation3D", LocationListener())
        drone.listen(OpenDJI.MODULE_FLIGHTCONTROLLER, "CompassHeading", YawListener())
        time.sleep(1)

        print("🚀 מנסה להמריא...")
        result = drone.takeoff(True)
        print(f"🛫 תוצאה: {result}")
        if "success" not in result.lower():
            print("❌ ההמראה נכשלה!")
            drone.__exit__(None, None, None)
            return None
        time.sleep(5)
        return drone
    except Exception as e:
        print(f"❌ שגיאה בחיבור או המראה: {e}")
        return None

# פונקציה לשליטה בגובה עם PID
def altitude_control_with_pid(drone, target_altitude):
    pid = PIDController(Kp=0.2, Ki=0.02, Kd=1.0, target=target_altitude)  # כוונון מחדש
    stabilized = False
    start_time = time.time()
    while not stabilized:
        if time.time() - start_time > 30:
            print("❌ לא הצליח להתייצב בגובה היעד תוך זמן סביר!")
            return False
        print(f"🛰 גובה נוכחי: {current_data['altitude']:.2f} מטרים")
        du = pid.update(current_data["altitude"])
        drone.move(0.0, du, 0.0, 0.0)
        time.sleep(0.03)  # הפחתה ל-0.03 שניות
        if abs(current_data["altitude"] - target_altitude) < 0.05:
            stabilized = True
    print(f"✅ התייצב בגובה {target_altitude} מטר")
    # עצירה קצרה עם בקרת גובה נוספת כדי להבטיח יציבות
    for _ in range(20):
        du = pid.update(current_data["altitude"])
        drone.move(0.0, du, 0.0, 0.0)
        time.sleep(0.03)
    return True

# פונקציה לסיבוב הרחפן לכיוון היעד
def rotate_to_target(drone, target_yaw):
    print(f"🔄 מסובב לכיוון {target_yaw:.2f} מעלות")
    start_time = time.time()
    while True:
        if time.time() - start_time > 30:
            print("❌ לא הצליח לסובב לכיוון היעד תוך זמן סביר!")
            return False
        current_yaw = current_data["yaw"]
        error = (target_yaw - current_yaw + 180) % 360 - 180
        if abs(error) < 5:
            break
        rcw = 0.1 if error > 0 else -0.1
        drone.move(rcw, 0.0, 0.0, 0.0)
        time.sleep(0.03)  # הפחתה ל-0.03 שניות
    drone.move(0.0, 0.0, 0.0, 0.0)
    print("✅ פונה לכיוון היעד")
    return True

# פונקציה לתנועה ליעד
def move_to_target(drone, target_lat, target_lon):
    pid = PIDController(Kp=0.2, Ki=0.02, Kd=1.0, target=2.0)  # כוונון מחדש
    print("➡️ נע לעבר היעד")
    start_time = time.time()
    while True:
        if time.time() - start_time > 60:
            print("❌ לא הצליח להגיע ליעד תוך זמן סביר!")
            return False
        # חישוב המרחק ליעד
        distance = math.sqrt((target_lat - current_data["latitude"])**2 + 
                            (target_lon - current_data["longitude"])**2)
        print(f"מרחק ליעד: {distance*100000:.2f} מטר, גובה נוכחי: {current_data['altitude']:.2f} מטר")

        # עצירה מוקדמת במרחק של כ-2 מטר
        if distance < STOP_DISTANCE_THRESHOLD:
            break

        # חישוב הזווית ליעד מחדש כדי לתקן מסלול
        bearing = calculate_bearing(current_data["latitude"], current_data["longitude"], 
                                    target_lat, target_lon)
        current_yaw = current_data["yaw"]
        yaw_error = (bearing - current_yaw + 180) % 360 - 180
        rcw = 0.05 if yaw_error > 0 else -0.05 if yaw_error < 0 else 0.0

        # הפחתת המהירות ככל שמתקרבים
        speed = min(0.1, max(0.02, distance * 100000 * 0.01))  # מהירות דינמית לפי המרחק

        # בקרת גובה ותנועה
        du = pid.update(current_data["altitude"])
        drone.move(rcw, du, 0.0, speed)
        time.sleep(0.03)  # הפחתה ל-0.03 שניות
    drone.move(0.0, 0.0, 0.0, 0.0)
    print(f"✅ עצר במרחק {distance*100000:.2f} מטר מהיעד")
    return True

# פונקציה ראשית
def main():
    drone = connect_and_takeoff()
    if drone:
        # ייצוב בגובה 2 מטר
        if not altitude_control_with_pid(drone, target_altitude=2.0):
            print("❌ נכשל בהמראה, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return
        # בדיקה נוספת ליציבות
        time.sleep(2)  # המתנה קצרה נוספת
        if abs(current_data["altitude"] - 2.0) > 0.1:
            print(f"⚠️ גובה לא יציב ({current_data['altitude']:.2f} מטר), ממשיך להתאם...")
            if not altitude_control_with_pid(drone, target_altitude=2.0):
                print("❌ נכשל בהתאמה, הסקריפט נעצר.")
                drone.land(True)
                time.sleep(5)
                drone.__exit__(None, None, None)
                return

        # חישוב הזווית ליעד
        bearing = calculate_bearing(current_data["latitude"], current_data["longitude"], 
                                   TARGET_LAT, TARGET_LON)
        print(f"📐 זווית ליעד: {bearing:.2f} מעלות")

        # סיבוב לכיוון היעד
        if not rotate_to_target(drone, bearing):
            print("❌ נכשל בסיבוב, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return

        # תנועה ליעד
        if not move_to_target(drone, TARGET_LAT, TARGET_LON):
            print("❌ נכשל בתנועה ליעד, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return

        # ירידה לגובה 1 מטר
        print("⬇️ יורד לגובה 1 מטר")
        if not altitude_control_with_pid(drone, target_altitude=1.0):
            print("❌ נכשל בירידה לגובה 1 מטר, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return

        # עצירה קצרה בגובה 1 מטר
        print("⏸ עצירה בגובה 1 מטר לפני נחיתה")
        time.sleep(5)

        # נחיתה
        print("🛬 נוחת...")
        drone.land(True)
        time.sleep(5)
        drone.unlisten(OpenDJI.MODULE_FLIGHTCONTROLLER, "AircraftLocation3D")
        drone.unlisten(OpenDJI.MODULE_FLIGHTCONTROLLER, "CompassHeading")
        drone.__exit__(None, None, None)
        print("🔌 נותק מהרחפן")

if __name__ == "__main__":
    main()
