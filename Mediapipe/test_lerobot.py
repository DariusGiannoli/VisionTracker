#!/usr/bin/env python3
"""
Script SIMPLE de contrôle de la pince du robot SO-101
Utilise la calibration sauvegardée par lerobot-calibrate
"""

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
import time
import json
from pathlib import Path

# Configuration
PORT = "/dev/tty.usbmodem58FD0170541"
ROBOT_ID = "dabrius"  # TON ID de calibration

# Positions en % (RANGE_M100_100 va de -100 à +100)
GRIPPER_OPEN = 100.0     # Complètement ouvert
GRIPPER_CLOSED = -100.0  # Complètement fermé
GRIPPER_MID = 0.0        # Position intermédiaire

def ouvrir_pince(bus):
    print("🔓 Ouverture...")
    bus.write("Goal_Position", "gripper", GRIPPER_OPEN)
    time.sleep(1)
    print("✓ Ouverte")

def fermer_pince(bus):
    print("🔒 Fermeture...")
    bus.write("Goal_Position", "gripper", GRIPPER_CLOSED)
    time.sleep(1)
    print("✓ Fermée")

def position_intermediaire(bus):
    print("↔️  Position intermédiaire...")
    bus.write("Goal_Position", "gripper", GRIPPER_MID)
    time.sleep(1)
    print("✓ Position intermédiaire")

# Classe simple pour la calibration (compatible avec l'API)
class CalibrationData:
    def __init__(self, data):
        self.id = data["id"]
        self.drive_mode = data["drive_mode"]
        self.homing_offset = data["homing_offset"]
        self.range_min = data["range_min"]
        self.range_max = data["range_max"]

class SimpleCalibration:
    def __init__(self, gripper_data):
        self.data = {"gripper": CalibrationData(gripper_data)}
    
    def __getitem__(self, key):
        return self.data.get(key)

if __name__ == "__main__":
    # Charger la calibration depuis le JSON
    calib_file = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower" / f"{ROBOT_ID}.json"
    
    print(f"Recherche calibration: {calib_file}")
    
    if calib_file.exists():
        with open(calib_file) as f:
            calib_data = json.load(f)
        
        if "gripper" in calib_data:
            print(f"✓ Calibration trouvée: {calib_data['gripper']}")
            calibration = SimpleCalibration(calib_data["gripper"])
        else:
            print("⚠️  Pas de calibration pour gripper, valeurs par défaut")
            calibration = SimpleCalibration({
                "id": 6,
                "drive_mode": 0,
                "homing_offset": 0,
                "range_min": 1999,
                "range_max": 3443
            })
    else:
        print(f"❌ Fichier non trouvé. Cherche dans:")
        print(f"   {calib_file.parent}/")
        if calib_file.parent.exists():
            for f in calib_file.parent.glob("*.json"):
                print(f"   - {f.name}")
        exit(1)
    
    # Connexion
    print(f"\nConnexion sur {PORT}...")
    
    bus = FeetechMotorsBus(
        port=PORT,
        motors={"gripper": Motor(6, "sts3215", MotorNormMode.RANGE_M100_100)},
        calibration=calibration
    )
    
    try:
        bus.connect()
        print("✓ Connecté !\n")
        
        # Position actuelle
        pos = bus.read("Present_Position", "gripper")
        print(f"Position: {pos:.1f}%\n")
        
        # Test
        ouvrir_pince(bus)
        time.sleep(2)
        fermer_pince(bus)
        time.sleep(2)
        position_intermediaire(bus)
        time.sleep(1)
        ouvrir_pince(bus)
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrompu")
    except Exception as e:
        print(f"\n❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nDéconnexion...")
        bus.disconnect()
        print("✓ Déconnecté")