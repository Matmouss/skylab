#!/bin/bash
set -e

SERVICE_NAME="drone-main"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
MAIN_FILE="$PROJECT_DIR/main.py"
PYTHON_BIN="/usr/bin/python3"
USER_NAME="$(whoami)"
SUDOERS_FILE="/etc/sudoers.d/drone-pppd"

# Vérification de la présence du fichier principal.
if [ ! -f "$MAIN_FILE" ]; then
    echo "ERREUR: main.py introuvable dans $PROJECT_DIR"
    exit 1
fi

# Détection de pppd, nécessaire pour la connexion 4G.
PPPD_PATH="$(command -v pppd || true)"

if [ -z "$PPPD_PATH" ]; then
    echo "ERREUR: pppd introuvable. Installation requise:"
    echo "sudo apt install ppp"
    exit 1
fi

# Autorisation de pppd sans mot de passe pour éviter un blocage sous systemd.
echo "${USER_NAME} ALL=(root) NOPASSWD: ${PPPD_PATH}" | sudo tee "$SUDOERS_FILE" > /dev/null
sudo chmod 440 "$SUDOERS_FILE"
sudo visudo -cf "$SUDOERS_FILE"

# Création du service systemd avec redémarrage automatique.
sudo tee /etc/systemd/system/${SERVICE_NAME}.service > /dev/null <<EOF
[Unit]
Description=Drone main.py service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER_NAME}
WorkingDirectory=${PROJECT_DIR}
ExecStart=${PYTHON_BIN} ${MAIN_FILE}
Restart=always
RestartSec=3
KillSignal=SIGTERM
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

# Activation et démarrage du service.
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}.service
sudo systemctl restart ${SERVICE_NAME}.service

echo "Service installé et démarré: ${SERVICE_NAME}"
echo "Statut: sudo systemctl status ${SERVICE_NAME}"
echo "Logs: journalctl -u ${SERVICE_NAME} -f"
echo "Redémarrage: sudo systemctl restart ${SERVICE_NAME}"
echo "Arrêt: sudo systemctl stop ${SERVICE_NAME}"
echo "Désactivation: sudo systemctl disable ${SERVICE_NAME}"