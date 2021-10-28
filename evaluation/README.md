```
python3 -m venv env
source env/bin/activate

jupyter nbextension install --py --symlink --sys-prefix ipyleaflet
jupyter nbextension enable ipyleaflet --py --sys-prefix

jupyter lab
```
