#!/usr/bin/env python


def to_decimal(value: str, indicator: str) -> float:
    # ddmm.mmmm / dddmm.mmmm -> decimal degrees
    dot = value.index(".")
    decimal = int(value[: dot - 2]) + float(value[dot - 2 :]) / 60
    return -decimal if indicator in ("S", "W") else decimal


if __name__ == "__main__":
    with open("../get_data/data.txt", "r") as file:
        for line in file:
            header, _, data = line.partition("$")[2].partition(",")
            if header != "GNRMC":
                continue
            sections = data.split(",")
            if sections[1] != "A":  # no fix
                continue
            time = sections[0]
            latitude = to_decimal(sections[2], sections[3])
            longitude = to_decimal(sections[4], sections[5])
            utm_zone = int((longitude + 180) / 6) + 1

            print(f"time := {time}  | latitude := {latitude:.8f} | longitude := {longitude:.8f} | utm zone := {utm_zone}")
