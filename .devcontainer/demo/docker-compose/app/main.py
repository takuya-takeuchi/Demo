import redis
import time
import os

REDIS_HOST = os.getenv("REDIS_HOST", "redis")
REDIS_PORT = os.getenv("REDIS_PORT", 6379)

def main():
    print(f"Connecting to Redis at {REDIS_HOST}:{REDIS_PORT}...")
    
    try:
        r = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)
        if r.ping():
            print("Successfully connected to Redis!")

        print("Setting key 'my_key' with value 'Hello Redis!'")
        r.set("my_key", "Hello Redis!")

        value = r.get("my_key")
        print(f"Retrieved value: {value}")

        while True:
            time.sleep(5)
            r.set("heartbeat", time.strftime("%H:%M:%S"))
            print(f"Heartbeat set: {r.get('heartbeat')}")

    except redis.exceptions.ConnectionError as e:
        print(f"Could not connect to Redis: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()