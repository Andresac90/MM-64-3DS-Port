#!/usr/bin/env python3
"""make_placeholder_art.py - original placeholder HOME Menu art for the MM port: port/icon.png (48x48) and
port/banner.bnr (bannertool's standard flat picture banner, the format the real HOME Menu shows fine).
A night sky with a plain moon and the title text (Pillow's default font) - no Nintendo artwork.
Needs Pillow and bannertool (PATH or $BANNERTOOL)."""
import math, os, random, shutil, struct, subprocess, sys, tempfile, wave
from PIL import Image, ImageDraw, ImageFilter, ImageFont

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def sky(w, h, seed=7):
    img = Image.new("RGBA", (w, h))
    d = ImageDraw.Draw(img)
    for y in range(h):
        t = y / h
        d.line([(0, y), (w, y)], fill=(int(30 + 60 * t), int(12 + 20 * t), int(60 + 40 * t), 255))
    random.seed(seed)
    for _ in range(w * h // 900):
        d.point((random.randrange(w), random.randrange(h)), fill=(255, 250, 220, 255))
    return img


def moon(img, cx, cy, r):
    glow = Image.new("RGBA", img.size, (0, 0, 0, 0))
    ImageDraw.Draw(glow).ellipse((cx - r * 1.6, cy - r * 1.6, cx + r * 1.6, cy + r * 1.6), fill=(255, 170, 90, 110))
    img.alpha_composite(glow.filter(ImageFilter.GaussianBlur(r * 0.5)))
    d = ImageDraw.Draw(img)
    d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(240, 170, 95, 255))
    for ox, oy, cr in ((-0.35, -0.25, 0.22), (0.3, 0.1, 0.16), (-0.05, 0.4, 0.12), (0.4, -0.4, 0.1)):
        d.ellipse((cx + ox * r - cr * r, cy + oy * r - cr * r, cx + ox * r + cr * r, cy + oy * r + cr * r),
                  fill=(210, 140, 75, 255))


def font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def chime(path):
    rate, n = 32000, int(32000 * 1.0)
    with wave.open(path, "wb") as w:
        w.setnchannels(1), w.setsampwidth(2), w.setframerate(rate)
        frames = bytearray()
        for i in range(n):
            t = i / rate
            f = 392.0 if t < 0.35 else 311.13
            v = 0.3 * math.sin(2 * math.pi * f * t) * math.exp(-3 * (t % 0.35 if t < 0.35 else t - 0.35))
            frames += struct.pack("<h", int(v * 32767))
        w.writeframes(bytes(frames))


def main():
    icon = sky(48 * 4, 48 * 4)
    moon(icon, 96, 96, 60)
    icon.resize((48, 48), Image.LANCZOS).save(os.path.join(REPO, "port/icon.png"))
    W, H = 512, 256
    ban = sky(W, H, 3)
    moon(ban, 110, 128, 78)
    d = ImageDraw.Draw(ban)
    for text, size, y, col in (("The Legend of Zelda:", 24, 72, (235, 225, 200)), ("Majora's Mask", 40, 104, (255, 215, 120)),
                               ("N64 3DS Port", 22, 160, (220, 215, 235))):
        while size > 10 and d.textbbox((0, 0), text, font=font(size))[2] > W - 214:  # fit the width
            size -= 1
        d.text((206, y), text, font=font(size), fill=col)
    tool = os.environ.get("BANNERTOOL") or shutil.which("bannertool")
    if not tool:
        sys.exit("bannertool not found")
    with tempfile.TemporaryDirectory() as tmp:
        png, wav = os.path.join(tmp, "b.png"), os.path.join(tmp, "b.wav")
        ban.convert("RGB").resize((256, 128), Image.LANCZOS).save(png)
        ban.convert("RGB").resize((256, 128), Image.LANCZOS).save(os.path.join(REPO, "port/banner_preview.png"))
        chime(wav)
        subprocess.run([tool, "makebanner", "-i", png, "-a", wav, "-o", os.path.join(REPO, "port/banner.bnr")], check=True,
                       capture_output=True)
    print("port/icon.png, port/banner.bnr, port/banner_preview.png")


if __name__ == "__main__":
    main()
