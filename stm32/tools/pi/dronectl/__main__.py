"""Permite rodar ``python -m dronectl`` a partir de tools/pi."""

import sys

from .cli import main

sys.exit(main())
