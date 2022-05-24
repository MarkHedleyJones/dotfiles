#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd )"
LOCAL_BIN="${HOME}/.local/bin"

if [ ! -L ${HOME}/.config/i3 ]; then
    echo "Linking ~/.config/i3 to ${SCRIPT_DIR}/i3"
    mv ${HOME}/.config/i3 ${HOME}/.config/i3_original
    ln -s ${SCRIPT_DIR}/i3 ${HOME}/.config/i3
fi

if [ ! -d ${LOCAL_BIN} ]; then
    echo "Creating a local binary folder at: ${LOCAL_BIN}"
    mkdir ${LOCAL_BIN}
fi

for filename in $(ls ${SCRIPT_DIR}/bin); do
    if [ ! -L ${LOCAL_BIN}/${filename} ]; then
        echo "Linking ${filename} to ${LOCAL_BIN}/${filename}"
        ln -s ${SCRIPT_DIR}/bin/${filename} ${LOCAL_BIN}/${filename}
    fi
done

if [ ! -L ${HOME}/.screenlayout ]; then
    echo "Linking ~/.screenlayout to ${SCRIPT_DIR}/screenlayout"
    ln -s ${SCRIPT_DIR}/.screenlayout ${HOME}
fi
