"""터미널 표 정렬 — 한글은 두 칸을 차지함.

파이썬의 `f"{x:>9}"` 는 **글자 수**로 셈. 터미널은 **칸 수**로 그림. 한글 한 글자가
두 칸이라 둘이 어긋남.

    f"{'최소':>9}"      9글자로 맞춤 -> 화면에서는 11칸
    f"{-117.07:9.2f}"   9글자        -> 화면에서도 9칸

머리글이 한글이고 아래 값이 숫자인 표에서 **머리글만 두 칸씩 밀림.**

    관절                최소        지금        최대
    hip_pitch         -117.07    -21.35    -21.18

`unicodedata.east_asian_width` 로 칸 수를 세서 채움.
"""

from __future__ import annotations

import unicodedata

WIDE = ("W", "F")
"""한 글자가 두 칸인 부류. Wide 와 Fullwidth.

한글·한자·가나가 여기 들어감. `A`(Ambiguous)는 글꼴에 따라 달라 한 칸으로 봄 --
숫자와 라틴 문자는 어차피 한 칸이라 영향이 없음.
"""


def width(text: str) -> int:
    """화면에서 차지하는 칸 수."""
    return sum(2 if unicodedata.east_asian_width(c) in WIDE else 1 for c in str(text))


def cell(text: str, size: int, *, align: str = ">") -> str:
    """칸 수 기준으로 채움. `align` 은 `<` 왼쪽, `>` 오른쪽.

    이미 넘치면 자르지 않고 그대로 냄 -- 표가 조금 밀리는 것보다 값이 잘리는 것이
    나쁨.
    """
    text = str(text)
    pad = max(0, int(size) - width(text))
    return (text + " " * pad) if align == "<" else (" " * pad + text)


def header(*columns) -> str:
    """`(이름, 칸수)` 또는 `(이름, 칸수, 정렬)` 을 받아 머리글 한 줄로.

        header(("관절", 10, "<"), ("최소", 9), ("최대", 9))
    """
    parts = []
    for column in columns:
        name, size = column[0], column[1]
        align = column[2] if len(column) > 2 else ">"
        parts.append(cell(name, size, align=align))
    return " ".join(parts)
