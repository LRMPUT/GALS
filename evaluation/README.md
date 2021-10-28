## Setup for the first time
```bash
python3 -m venv env
source env/bin/activate
pip install pip --upgrade
pip install -r requirements.txt
jupyter nbextension install --py --symlink --sys-prefix ipyleaflet
jupyter nbextension enable ipyleaflet --py --sys-prefix
```
## Run
```bash
source env/bin/activate
jupyter lab
```
## Notes

- Conversion to UTM introduces scale error, so it's better to convert to ECEF / ENU for ATE evaluation