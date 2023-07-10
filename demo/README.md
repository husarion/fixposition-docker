# Fixposition Vistion-RTK-2 demo

Pulling Docker images:

```bash
docker compose pull
```

Connect to your device either using WiFi or Ethernet (when using Ethernet connection it will be necessary to adjust IP address configuration in the `tcp.yaml`, for example for Panther you should use `10.15.20.20`).

Execution: 

```bash
docker compose up
```

In your browser (Chrome/Chromium or related browsers are recommended) go to Foxglove:
`localhost:8080`

Now click the plus sign next to the `Data source` in the left top corner, select `Open connection` and set the `WebSocket URL` to `ws://localhost:9090`, finally click `Open`.