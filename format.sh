#!/bin/bash
find . -name "*py"|xargs autoflake --remove-all-unused-imports -i --ignore-init-module-imports
isort . 
black .
flake8 .

