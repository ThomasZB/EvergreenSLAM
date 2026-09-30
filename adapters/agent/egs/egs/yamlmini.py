"""The flat YAML subset memory/ files use (place.yaml, node.yaml, zone.yaml).

One `key: value` per line. Values: int, float, true/false, null/~, bare or quoted strings, and
flow lists of those (`[a, b, 1.5]`); for the keys a caller names in `nested_keys` (zone.yaml's
`polygon`) the items may themselves be flow lists of scalars (`[[1.0, -1.0], [3.0, -1.0]]`, one
level). `#` starts a comment outside quotes. Nested maps,
deeper lists, block lists (`- a`), anchors and multi-line strings are rejected with
YamlSubsetError: egs never guesses. Writing drops comments.
"""

import json
import re

_KEY = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*:(.*)$")
_INT = re.compile(r"^[-+]?[0-9]+$")
_FLOAT = re.compile(r"^[-+]?([0-9]+\.[0-9]*|\.[0-9]+|[0-9]+)([eE][-+]?[0-9]+)?$")
_SPECIAL = set(":#[]{},&*!|>'\"%@`")


class YamlSubsetError(ValueError):
    pass


def _strip_comment(text):
    quote, escaped = None, False
    for i, c in enumerate(text):
        if quote:
            if escaped:
                escaped = False
            elif c == "\\" and quote == '"':
                escaped = True
            elif c == quote:
                quote = None
        elif c in "'\"":
            quote = c
        elif c == "#" and (i == 0 or text[i - 1] in " \t"):
            return text[:i]
    return text


def _split_flow(body, lineno, allow_nested=False):
    items, cur, quote, depth, i = [], "", None, 0, 0
    while i < len(body):
        c = body[i]
        if quote:
            cur += c
            if c == "\\" and quote == '"' and i + 1 < len(body):
                cur += body[i + 1]
                i += 1
            elif c == quote:
                quote = None
        elif c in "'\"":
            quote = c
            cur += c
        elif c == "[" and allow_nested and depth == 0 and not cur.strip():
            depth = 1
            cur += c
        elif c == "]" and depth == 1:
            depth = 0
            cur += c
        elif c in "[]{}":
            raise YamlSubsetError("line %d: nested collections are outside the subset" % lineno)
        elif c == "," and depth == 0:
            items.append(cur)
            cur = ""
        else:
            cur += c
        i += 1
    if quote:
        raise YamlSubsetError("line %d: unterminated quote" % lineno)
    if depth:
        raise YamlSubsetError("line %d: unclosed inner list" % lineno)
    items.append(cur)
    items = [s.strip() for s in items]
    if items == [""]:
        return []
    if any(s == "" for s in items):
        raise YamlSubsetError("line %d: empty list item" % lineno)
    return items


def _scalar(text, lineno):
    text = text.strip()
    if text == "" or text in ("~", "null", "Null", "NULL"):
        return None
    if text in ("true", "True", "TRUE"):
        return True
    if text in ("false", "False", "FALSE"):
        return False
    if text[0] == '"':
        if len(text) < 2 or text[-1] != '"':
            raise YamlSubsetError("line %d: bad double-quoted string" % lineno)
        try:
            return json.loads(text)
        except ValueError:
            raise YamlSubsetError("line %d: bad double-quoted string" % lineno)
    if text[0] == "'":
        if len(text) < 2 or text[-1] != "'":
            raise YamlSubsetError("line %d: bad single-quoted string" % lineno)
        return text[1:-1].replace("''", "'")
    if _INT.match(text):
        return int(text)
    if _FLOAT.match(text):
        return float(text)
    if text[0] in "&*!|>{-" or text.startswith("- "):
        raise YamlSubsetError("line %d: %r is outside the subset" % (lineno, text))
    return text


def loads(text, nested_keys=()):
    """Returns an ordered dict of the file's keys. Raises YamlSubsetError outside the subset."""
    out = {}
    for lineno, raw in enumerate(text.splitlines(), 1):
        if raw.strip() in ("---", "..."):
            continue
        line = _strip_comment(raw).rstrip()
        if not line.strip():
            continue
        if line[0] in " \t":
            raise YamlSubsetError("line %d: indentation (nested map or block list)" % lineno)
        m = _KEY.match(line)
        if not m:
            raise YamlSubsetError("line %d: expected 'key: value'" % lineno)
        key, value = m.group(1), m.group(2).strip()
        if key in out:
            raise YamlSubsetError("line %d: duplicate key %r" % (lineno, key))
        if value.startswith("["):
            if not value.endswith("]"):
                raise YamlSubsetError("line %d: flow list must close on the same line" % lineno)
            nested = key in nested_keys
            out[key] = [_item(s, lineno) for s in _split_flow(value[1:-1], lineno, nested)]
        elif value == "":
            raise YamlSubsetError("line %d: %r has no value (nested map?)" % (lineno, key))
        else:
            out[key] = _scalar(value, lineno)
    return out


def _item(text, lineno):
    if not text.startswith("["):
        return _scalar(text, lineno)
    if not text.endswith("]"):
        raise YamlSubsetError("line %d: text after an inner list" % lineno)
    return [_scalar(s, lineno) for s in _split_flow(text[1:-1], lineno)]


def _dump_item(v):
    if isinstance(v, (list, tuple)):
        return "[%s]" % ", ".join(_dump_scalar(x) for x in v)
    return _dump_scalar(v)


def _dump_scalar(v):
    if v is None:
        return "null"
    if v is True:
        return "true"
    if v is False:
        return "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return repr(v)
    s = str(v)
    needs_quote = (
        s == ""
        or s != s.strip()
        or any(c in _SPECIAL for c in s)
        or s[0] in "-?"
        or "\n" in s
        or _scalar_is_not_string(s)
    )
    return json.dumps(s, ensure_ascii=False) if needs_quote else s


def _scalar_is_not_string(s):
    try:
        return not isinstance(_scalar(s, 0), str)
    except YamlSubsetError:
        return True


def dumps(data, order=()):
    """Keys in `order` first, then the rest in insertion order."""
    keys = [k for k in order if k in data] + [k for k in data if k not in order]
    lines = []
    for k in keys:
        v = data[k]
        if isinstance(v, (list, tuple)):
            lines.append("%s: [%s]" % (k, ", ".join(_dump_item(x) for x in v)))
        else:
            lines.append("%s: %s" % (k, _dump_scalar(v)))
    return "\n".join(lines) + "\n"
