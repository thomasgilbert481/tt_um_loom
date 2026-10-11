"""Constant expressions for the Loom assembler.

Grammar, lowest precedence first::

    or    := and    ('|' and)*
    and   := shift  ('&' shift)*
    shift := add    (('<<' | '>>') add)*
    add   := mul    (('+' | '-') mul)*
    mul   := unary  (('*' | '/') unary)*
    unary := ('-' | '+') unary | primary
    prim  := NUMBER | 'pin' '(' NAME ')' | FUNC '(' or (',' or)* ')' | NAME
           | '(' or ')'

``lo8`` and ``hi8`` take one value; ``min`` and ``max`` two or more.
``pin(NAME)`` is the index of a pin, a ``.pins`` alias or an isa.yaml pin
name, so a program can derive masks and an ``OUT`` group from its pins and
be re-pinned without editing (D-035). It is a function rather than a bare
name so that a pin alias never shadows a label of the same name.
Values are arbitrary-precision Python integers; the range check happens where
the value is used (``tools.loomisa`` checks instruction fields).
"""

from __future__ import annotations

from typing import Callable, List, Set

from .lexer import NAME, NUMBER, PUNCT, Token

FUNCTIONS = {
    "lo8": lambda v: v & 0xFF,
    "hi8": lambda v: (v >> 8) & 0xFF,
}

#: Functions of two or more values.
FUNCTIONS_N = {
    "min": min,
    "max": max,
}

_MAX_SHIFT = 64


class ExprError(Exception):
    """A malformed or unevaluable expression."""

    def __init__(self, message: str, token: "Token | None" = None):
        super().__init__(message)
        self.message = message
        self.token = token


class UnresolvedSymbol(ExprError):
    """A symbol the resolver does not know yet (a forward reference)."""

    def __init__(self, name: str, token: "Token | None" = None):
        super().__init__("unknown symbol '%s'" % name, token)
        self.name = name


Resolver = Callable[[str], int]


def _no_pins(name: str) -> int:
    raise ExprError("pin(%s): no pins are known here" % name)


def _trunc_div(a: int, b: int) -> int:
    """Integer division truncating towards zero, as C and most assemblers do."""
    quotient = abs(a) // abs(b)
    return -quotient if (a < 0) != (b < 0) else quotient


class _Parser:
    def __init__(self, tokens: List[Token], resolve: Resolver, pin: Resolver = _no_pins):
        self.tokens = tokens
        self.resolve = resolve
        self.pin = pin
        self.pos = 0

    # -------------------------------------------------------------- helpers
    def peek(self) -> "Token | None":
        return self.tokens[self.pos] if self.pos < len(self.tokens) else None

    def take(self) -> Token:
        token = self.tokens[self.pos]
        self.pos += 1
        return token

    def accept_punct(self, *texts: str) -> "Token | None":
        token = self.peek()
        if token is not None and token.kind == PUNCT and token.text in texts:
            return self.take()
        return None

    # --------------------------------------------------------------- levels
    def parse(self) -> int:
        if not self.tokens:
            raise ExprError("expected an expression")
        value = self.or_()
        extra = self.peek()
        if extra is not None:
            raise ExprError("unexpected '%s' after the expression" % extra.text, extra)
        return value

    def or_(self) -> int:
        value = self.and_()
        while self.accept_punct("|"):
            value |= self.and_()
        return value

    def and_(self) -> int:
        value = self.shift()
        while self.accept_punct("&"):
            value &= self.shift()
        return value

    def shift(self) -> int:
        value = self.add()
        while True:
            op = self.accept_punct("<<", ">>")
            if op is None:
                return value
            amount = self.add()
            if amount < 0:
                raise ExprError("shift amount %d is negative" % amount, op)
            if amount > _MAX_SHIFT:
                raise ExprError("shift amount %d is too large" % amount, op)
            value = value << amount if op.text == "<<" else value >> amount

    def add(self) -> int:
        value = self.mul()
        while True:
            op = self.accept_punct("+", "-")
            if op is None:
                return value
            rhs = self.mul()
            value = value + rhs if op.text == "+" else value - rhs

    def mul(self) -> int:
        value = self.unary()
        while True:
            op = self.accept_punct("*", "/")
            if op is None:
                return value
            rhs = self.unary()
            if op.text == "*":
                value = value * rhs
            else:
                if rhs == 0:
                    raise ExprError("division by zero", op)
                value = _trunc_div(value, rhs)

    def unary(self) -> int:
        op = self.accept_punct("-", "+")
        if op is not None:
            value = self.unary()
            return -value if op.text == "-" else value
        return self.primary()

    def primary(self) -> int:
        token = self.peek()
        if token is None:
            raise ExprError("expected a value at the end of the expression")
        if token.kind == NUMBER:
            self.take()
            return int(token.value or 0)
        if token.kind == NAME:
            self.take()
            nxt = self.peek()
            if nxt is not None and nxt.is_punct("(") and token.text.lower() == "pin":
                self.take()
                arg = self.peek()
                if arg is None or arg.kind != NAME:
                    raise ExprError("pin() takes a pin name", token)
                self.take()
                closing = self.peek()
                if closing is None or not closing.is_punct(")"):
                    raise ExprError("missing ')' after pin(", token)
                self.take()
                try:
                    return self.pin(arg.text)
                except ExprError as exc:
                    raise ExprError(exc.message, arg) from None
            if nxt is not None and nxt.is_punct("("):
                name = token.text.lower()
                func, many = FUNCTIONS.get(name), FUNCTIONS_N.get(name)
                if func is None and many is None:
                    raise ExprError("unknown function '%s'" % token.text, token)
                self.take()
                values = [self.or_()]
                while self.accept_punct(","):
                    values.append(self.or_())
                closing = self.peek()
                if closing is None or not closing.is_punct(")"):
                    raise ExprError("missing ')' after %s(" % token.text, token)
                self.take()
                if func is not None:
                    if len(values) != 1:
                        raise ExprError("%s() takes one value" % name, token)
                    return func(values[0])
                if len(values) < 2:
                    raise ExprError("%s() takes two or more values" % name, token)
                return many(values)
            return self.resolve(token.text)
        if token.is_punct("("):
            self.take()
            value = self.or_()
            closing = self.peek()
            if closing is None or not closing.is_punct(")"):
                raise ExprError("missing ')'", token)
            self.take()
            return value
        raise ExprError("'%s' cannot start an expression" % token.text, token)


def evaluate(tokens: List[Token], resolve: Resolver, pin: Resolver = _no_pins) -> int:
    """Evaluate ``tokens`` as a constant expression.

    ``resolve(name)`` returns the value of a symbol or raises
    :class:`UnresolvedSymbol`; ``pin(name)`` returns a pin's index for
    ``pin(NAME)`` or raises :class:`ExprError`.
    """
    return _Parser(list(tokens), resolve, pin).parse()


def symbols_in(tokens: List[Token]) -> Set[str]:
    """Names the expression reads, ignoring function names and the pin name
    inside ``pin(...)``."""
    found: Set[str] = set()
    for index, token in enumerate(tokens):
        if token.kind != NAME:
            continue
        nxt = tokens[index + 1] if index + 1 < len(tokens) else None
        if nxt is not None and nxt.is_punct("("):
            continue
        if (index >= 2 and tokens[index - 1].is_punct("(")
                and tokens[index - 2].kind == NAME and tokens[index - 2].text.lower() == "pin"):
            continue
        found.add(token.text)
    return found
