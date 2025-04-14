@echo off
setlocal

REM === CONFIGURAZIONE ===
set BOARD_FQBN=esp32:esp32:esp32
set COM_PORT=COM6
set SKETCH_PATH=%~dp0
set DATA_DIR=%SKETCH_PATH%\data

REM === Verifica presenza cartella data ===
if not exist "%DATA_DIR%" (
    echo [ERRORE] La cartella "data" non esiste nella directory dello sketch!
    pause
    exit /b 1
)

REM === Crea filesystem image ===
echo [1/3] Creazione immagine SPIFFS...
arduino-cli compile --fqbn %BOARD_FQBN% "%SKETCH_PATH%"
arduino-cli upload --fqbn %BOARD_FQBN% --port %COM_PORT% --input-dir "%DATA_DIR%" --filesystem spiffs

if %errorlevel% neq 0 (
    echo [ERRORE] Caricamento SPIFFS fallito.
    pause
    exit /b 1
)

echo [3/3] Upload completato con successo!
pause