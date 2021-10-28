# Setup for first time
```
python3 -m venv env
source env/bin/activate

jupyter nbextension install --py --symlink --sys-prefix ipyleaflet
jupyter nbextension enable ipyleaflet --py --sys-prefix
```

# Run
```
source env/bin/activate
jupyter lab
```

## Notes

- Conversion to UTM introduces scale error, so it's better to convert to ECEF / ENU for ATE evaluation