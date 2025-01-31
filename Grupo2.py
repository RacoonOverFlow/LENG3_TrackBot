from machine import UART, Pin
from mqtt_as import MQTTClient, config
import uasyncio as asyncio
import json

# Configuração do ThingsBoard (MQTT)
config["server"] = "mqtt.eu.thingsboard.cloud"
config["user"] = "L964xLPjMGRVxmzP0mCx"  # Token do dispositivo como usuário
               
config["ssid"] = "LENG3a"
config["wifi_pw"] = "nefsloiat"

# Configuração da UART para comunicação com o Arduino
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))


ultimo_lap = -1
ultimo_lap_time = -1
ultimo_total_time = -1

async def main(client):
    global ultimo_lap, ultimo_lap_time, ultimo_total_time

    while True:
        if uart.any():
            try:
                line = uart.readline().decode("utf-8").strip()
                print(f"Dadod recebidos: {line}")

                if line.startswith("{") and line.endswith("}"):
                    data = json.loads(line)

                    lap = data.get("lap", 0)
                    lap_time = data.get("lapTime", 0.0)
                    total_time = data.get("totalTime", 0.0)

                    if (lap != ultimo_lap) or (lap_time != ultimo_lap_time) or (total_time != ultimo_total_time):
                        telemetry_data = {
                            "lap": lap,
                            "lap_time": lap_time,
                            "total_time": total_time
                        }
                        await client.publish("v1/devices/me/telemetry", json.dumps(telemetry_data), qos=1)
                        print(f"Dados enviados ao ThingsBoard: {telemetry_data}")

                        ultimo_lap = lap
                        ultimo_lap_time = lap_time
                        ultimo_total_time = total_time

            except Exception as e:
                print(f"Erro ao processar dados: {e}")

        await asyncio.sleep(1)

async def mqtt_connect():
    while True:
        try:
            print("A conectar ao ThingsBoard...")
            await client.connect()
            print("Conectado ao ThingsBoard")
            break
        except Exception as e:
            print(f"Erro de conexão MQTT: {e}, a tentar novamente em 10s...")
            await asyncio.sleep(10)

MQTTClient.DEBUG = True
client = MQTTClient(config)

try:
    asyncio.run(mqtt_connect())
    asyncio.run(main(client))
finally:
    client.close()