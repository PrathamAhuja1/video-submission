import pygame
import socket
import time

# ================= CONFIG =================
RECEIVER_IP = "192.168.11.2"
UDP_PORT = 8080

HEAVE_NEUTRAL = 1500
HEAVE_NEUTRAL_FR = 1500

SURGE1_NEUTRAL = 1530
SURGE2_NEUTRAL = 1500

SWAY_NEUTRAL = 1500

# Global limits (surge, sway)
PWM_MIN = 1300
PWM_MAX = 1700

# Heave-only limits
HEAVE_MIN = 1300
HEAVE_MAX = 1700

DEADBAND = 0.05
POLL_DELAY = 0.01
# =========================================


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def create_payload(h1, h2, s1, s2, h3, h4, sw1, sw2):
    return f"{h1}/{h2}/{s1}/{s2}/{h3}/{h4}/{sw1}/{sw2}/\n"


def create_neutral_payload():
    return "1500/1500/1530/1500/1500/1480/1500/1500/\n"


def main():
    print("Initializing UDP sender")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print(" No joystick detected")
        return

    js = pygame.joystick.Joystick(0)
    js.init()

    print("Joystick:", js.get_name())

    sent_neutral = False

    neutral_payload = create_neutral_payload()
    last_payload = None

    print("neutral : " ,neutral_payload)

    sock.sendto(neutral_payload.encode(), (RECEIVER_IP, UDP_PORT))
    last_payload = neutral_payload

    try:
        while True:
            pygame.event.pump()

            # -------- DEFAULTS --------
            heave_1 = heave_2 = heave_3 = HEAVE_NEUTRAL
            heave_4 = HEAVE_NEUTRAL_FR

            surge_1 = SURGE1_NEUTRAL
            surge_2 = SURGE2_NEUTRAL

            sway_1 = sway_2 = SWAY_NEUTRAL

            active = False

            # -------- HEAVE (Left stick Y) --------
            axis_heave = js.get_axis(1)


            if abs(axis_heave) > DEADBAND:
                delta = int(axis_heave * 480)

                heave_3 = clamp(
                    1500 - delta, HEAVE_MIN, HEAVE_MAX
                )
                heave_1 = heave_4 =  clamp(
                    1500 + delta, HEAVE_MIN, HEAVE_MAX
                )
                heave_2 = clamp(
                    1500 - delta, HEAVE_MIN, HEAVE_MAX
                )
                active = True

            # -------- FORWARD / REVERSE (TRIGGERS) --------
            rt = (js.get_axis(5) + 1) / 2
            lt = (js.get_axis(2) + 1) / 2 

            forward = 0
            if rt > 0.05:
                forward = rt
                active = True
            elif lt > 0.05:
                forward = -lt
                active = True

            # -------- YAW (Right stick X) --------
            axis_yaw = js.get_axis(3)
            yaw = 0
            if abs(axis_yaw) > DEADBAND:
                yaw = axis_yaw
                active = True

            # -------- SURGE MIXER --------
            if active:
                surge_1 += int(forward * 300)
                surge_2 += int(forward * 300)

                surge_1 -= int(-yaw * 300)
                surge_2 -= int(+yaw * 300)

                surge_1 = clamp(surge_1, PWM_MIN, PWM_MAX)
                surge_2 = clamp(surge_2, PWM_MIN, PWM_MAX)

            # -------- SWAY (LB / RB) --------
            if js.get_button(4):
                sway_1 = sway_2 = PWM_MIN
                active = True
            elif js.get_button(5):
                sway_1 = sway_2 = PWM_MAX
                active = True

            # -------- PAYLOAD --------
            payload = (
                create_payload(
                    heave_1, heave_2,
                    surge_1, surge_2,
                    heave_3, heave_4,
                    sway_1, sway_2
                ) if active else create_neutral_payload()
            )

            
            if active :
                sock.sendto(payload.encode(), (RECEIVER_IP, UDP_PORT))
                print("Sent:", payload.strip())
                sent_neutral = False
            else : 
                if sent_neutral:
                    continue
                else:
                    payload = create_neutral_payload()
                    sock.sendto(payload.encode(), (RECEIVER_IP, UDP_PORT))
                    print("Sent:", payload.strip())
                    sent_neutral = True

            

            time.sleep(POLL_DELAY)

    except KeyboardInterrupt:
        print("\nStopping sender…")

    finally:
        sock.sendto(create_neutral_payload().encode(), (RECEIVER_IP, UDP_PORT))
        time.sleep(0.2)
        sock.close()
        pygame.quit()
        print("Sender shut down cleanly")


if __name__ == "__main__":
    main()
