# STACK-A-BOT
STACK-A-BOT: Smart Tech And Compact Knowledge A Better Optimization Tool.
----
This repository contains source code for the Capstone Project at WPI. 


# How to use

This codebase is meant to be run as a package, and so you need to run it from an outside directory

``` 
venv/
stack_a_bot/
    code
    main.py
    ...
```
run the main code as 
```
python -m stack_a_bot.main 
```
or if running tests
```
python -m stack_a_bot.Tests.{testfile_name}
```
# Debugging

VSCodes internal debugger won't work normally for this package setup, so we need to setup a launch.json to specifically debug our main. Use the provided .vscode/launch.json to launch main in debug mode.


# Style Guide

Google offers a great style guide for python code...

Please use their [docustring](https://google.github.io/styleguide/pyguide.html#383-functions-and-methods) format for commenting functions when possible. 

## Quick Tips

* Classes are PascalCase, functions and varables are snake_case
* internal variables are prepended with underscores, and internal functions with two-underscores (e.g "_lock")
* Every function should have a docustring if behavior is not obvious
* All function parameters should have type descriptors in header
* Try and keep lines to a max of 80 characters long. Not a hard/fast rule.