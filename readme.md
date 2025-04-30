# Project README

## Prerequisites

- **Webots** installed on your host machine. Download and install from [webots.dev](https://cyberbotics.com) or via your system package manager.
- **Docker** & **Docker Compose** installed.

---c

## 1. Configure Environment

1. Copy the example env file:
   ```bash
   cp .env.example .env
   ```

## 2. Verify that WEBOTS_HOME points to your local webots install

    defaults are /usr/local/webots,
    snap defaults are /snap/webots/current/usr/share/webots

## 3. Launch The Programs

    All necessary programs are defined in the docker-compose, run it with up to generate and run all containers
    ```bash
    docker-compose up --build
    ```

## 4. Access Front-end

    The front-end lives under http://localhost:5173

## 5. Stopping & Cleanup

    To stop all containers from running, and cleanup any closed containers, run:
    ```bash
    docker-compose down && docker-compose rm -f
    ```