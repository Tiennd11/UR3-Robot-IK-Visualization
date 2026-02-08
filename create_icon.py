"""Generate a professional blue robot head icon."""
from PIL import Image, ImageDraw, ImageFilter
import os

def draw_robot_icon(size):
    """Draw a professional blue robot head icon at given size."""
    img = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    s = size

    blue_main = (74, 174, 240)
    white = (255, 255, 255)
    white_soft = (245, 248, 252)
    eye_blue = (45, 130, 200)
    antenna_ball = (235, 242, 250)

    def sc(v):
        return v * s / 512

    # Drop shadow
    shadow_img = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow_img)
    sd.ellipse([sc(80), sc(250), sc(432), sc(475)], fill=(0, 0, 0, 25))
    if size >= 48:
        shadow_img = shadow_img.filter(ImageFilter.GaussianBlur(radius=max(1, sc(15))))
    img = Image.alpha_composite(img, shadow_img)
    d = ImageDraw.Draw(img)

    # Antennas
    ant_lx, ant_rx = sc(120), sc(392)
    ant_top, ant_bot = sc(60), sc(205)
    stem_w = max(2, int(sc(11)))
    ball_r = sc(24)
    hr = sc(6)

    for ax in [ant_lx, ant_rx]:
        d.line([(ax, ant_top + ball_r), (ax, ant_bot)], fill=blue_main, width=stem_w)
        d.ellipse([ax - ball_r, ant_top - ball_r * 0.2, ax + ball_r, ant_top + ball_r * 1.6], fill=antenna_ball)
        d.ellipse([ax - hr * 2, ant_top + sc(3), ax, ant_top + sc(3) + hr * 1.5], fill=white)

    # Head dome
    head_l, head_r = sc(75), sc(437)
    head_t, head_b = sc(148), sc(435)
    d.ellipse([head_l, head_t, head_r, head_b], fill=blue_main)

    # Head highlight
    hl_img = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    hd = ImageDraw.Draw(hl_img)
    hd.ellipse([head_l + sc(25), head_t + sc(12), head_r - sc(70), head_b - sc(90)], fill=(255, 255, 255, 28))
    if size >= 48:
        hl_img = hl_img.filter(ImageFilter.GaussianBlur(radius=max(1, sc(25))))
    img = Image.alpha_composite(img, hl_img)
    d = ImageDraw.Draw(img)

    # Ears
    ear_r = sc(30)
    ear_y = sc(318)
    d.ellipse([head_l - ear_r * 0.2, ear_y - ear_r, head_l + ear_r * 1.4, ear_y + ear_r], fill=blue_main)
    d.ellipse([head_r - ear_r * 1.4, ear_y - ear_r, head_r + ear_r * 0.2, ear_y + ear_r], fill=blue_main)

    # Face visor
    vis_l, vis_r = sc(138), sc(374)
    vis_t, vis_b = sc(268), sc(392)
    d.rounded_rectangle([vis_l, vis_t, vis_r, vis_b], radius=int(sc(62)), fill=white_soft)
    d.rounded_rectangle([vis_l, vis_t, vis_r, vis_b], radius=int(sc(62)), fill=None, outline=(195, 215, 235), width=max(1, int(sc(2))))

    # Eyes
    eye_r = sc(27)
    eye_y = sc(330)
    for ex in [sc(213), sc(299)]:
        d.ellipse([ex - eye_r, eye_y - eye_r, ex + eye_r, eye_y + eye_r], fill=eye_blue)
        eh, eo = sc(7), sc(8)
        d.ellipse([ex - eo - eh, eye_y - eo - eh, ex - eo + eh, eye_y - eo + eh], fill=white)

    return img


def create_icon():
    sizes = [256, 128, 64, 48, 32, 16]
    images = [draw_robot_icon(sz) for sz in sizes]
    base_dir = os.path.dirname(os.path.abspath(__file__))
    ico_path = os.path.join(base_dir, 'app_icon.ico')
    images[0].save(ico_path, format='ICO', sizes=[(sz, sz) for sz in sizes], append_images=images[1:])
    print(f"[OK] Icon saved: {ico_path}")
    return ico_path


if __name__ == '__main__':
    create_icon()
