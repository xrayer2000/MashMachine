import paho.mqtt.client as mqtt
import winsound
import time
import sys
from pathlib import Path

MQTT_HOST = "192.168.0.174"
MQTT_PORT = 1883

# Sound files are stored in:
# C:\Users\xehon\sounds
SOUNDS_DIR = Path(r"C:\Users\xehon\sounds")

start_time = time.time()

# One-time stage alarms
TOPIC_SOUNDS = {
    "mashTun/mash_instep_alarm": "0_maskningsVattentArUppeITemperatur.wav",
    "mashTun/lautering_alarm": "1_maskningenFardig_v2.wav",
    "mashTun/boil_alarm": "2_lakningenFardig_v2.wav",
    "mashTun/hops_alarm": "3_dagsAttLaggaIHumlen_v2.wav",
    "mashTun/whirlpool_alarm": "4_kokning_fardig_v2.wav",
    "mashTun/whirlpool_finished_alarm": "5_virvlingenArFarig.wav",
}

# Repeating "still not switched" reminders
REMINDER_SOUNDS = {
    "mashTun/mash_instep_reminder": "0.1_maskningsVattentArUppeITemperatur_ForHel.wav",
    "mashTun/lautering_reminder": "1.1_maskningenArFardigForHel.wav",
    "mashTun/boil_reminder": "2.1_lakningenArFardigForHel.wav",
    "mashTun/whirlpool_reminder": "4.1_kokningenArFardigForHel.wav",
    "mashTun/whirlpool_finished_reminder": "5.1_virvlingenArFarig_forHel.wav",
}

ALL_TOPICS = list(TOPIC_SOUNDS.keys()) + list(REMINDER_SOUNDS.keys())


def play_sound(sound_file):
    """Play a WAV file using Windows audio."""
    sound_file = Path(sound_file)

    if not sound_file.exists():
        print(
            f"Sound file not found: {sound_file}",
            file=sys.stderr
        )
        return

    print(f"Playing: {sound_file.name}")

    try:
        winsound.PlaySound(
            str(sound_file),
            winsound.SND_FILENAME
        )
    except RuntimeError as e:
        print(
            f"Failed to play {sound_file}: {e}",
            file=sys.stderr
        )


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT broker")
    else:
        print(
            f"Connect failed: {reason_code}",
            file=sys.stderr
        )
        return

    for topic in ALL_TOPICS:
        client.subscribe(topic)
        print(f"Subscribed to {topic}")


def on_disconnect(
    client,
    userdata,
    disconnect_flags,
    reason_code,
    properties
):
    print(
        f"Disconnected from broker: {reason_code}",
        file=sys.stderr
    )


def on_message(client, userdata, msg):
    payload = msg.payload.decode(errors="replace")
    elapsed = time.time() - start_time

    print(
        f"Got message on {msg.topic}: {payload} "
        f"(elapsed: {elapsed:.1f}s, retained: {msg.retain})"
    )

    # One-time alarms
    if msg.topic in TOPIC_SOUNDS:

        # Ignore retained messages replayed on connect/reconnect
        if msg.retain:
            print(
                "  -> Ignoring (retained message replayed on "
                "connect/reconnect)"
            )
            return

        if msg.topic == "mashTun/hops_alarm":
            should_play = payload in ("1", "2", "3")
        else:
            should_play = payload == "1"

        if should_play:
            sound_file = SOUNDS_DIR / TOPIC_SOUNDS[msg.topic]
            play_sound(sound_file)

    # Repeating reminders
    elif msg.topic in REMINDER_SOUNDS:

        # Reminders should never be retained
        if msg.retain:
            print("  -> Ignoring (unexpected retained reminder)")
            return

        if payload == "1":
            sound_file = SOUNDS_DIR / REMINDER_SOUNDS[msg.topic]
            play_sound(sound_file)


client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2
)

client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

client.reconnect_delay_set(
    min_delay=1,
    max_delay=30
)

client.connect_async(
    MQTT_HOST,
    MQTT_PORT,
    60
)

client.loop_forever(
    retry_first_connection=True
)