"""Steuerungsschicht: langlebiger Daemon mit Webserver.

Iteration 1 (read-only): Ein Roboter-Worker-Thread besitzt exklusiv die
Hexapod-Instanz und liest in fester Frequenz den Hardware-Zustand aus.
Der FastAPI-Server liest diesen Snapshot nur und kommandiert KEINE Bewegung.
"""
