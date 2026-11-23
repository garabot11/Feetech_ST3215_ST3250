"""Start the local Feetech STS desktop panel."""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description="Панель сервоприводов Feetech ST3215 и ST3250")
    parser.add_argument("--demo", action="store_true", help="открыть панель с виртуальными моторами")
    args = parser.parse_args()

    try:
        import tkinter as tk
    except ImportError:
        print("Для запуска нужен Tkinter. В Ubuntu/Debian установите пакет python3-tk.", file=sys.stderr)
        return 1

    from .ui import Panel

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f"Не удалось открыть окно. Запустите программу в графическом сеансе. {exc}", file=sys.stderr)
        return 1

    Panel(root, demo=args.demo)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
