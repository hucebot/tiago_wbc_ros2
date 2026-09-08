"""Smoke tests for the tiago_control_node package.

These guard against the failure mode where a module is committed truncated or
importing a name that no longer exists -- which is exactly how
``tiago_opensot_node`` stayed broken for months while still being a registered
entry point.

Two layers:

* :func:`test_source_files_parse` and :func:`test_intra_package_imports_resolve`
  need **no dependencies at all** (they work on the AST), so they run in any CI,
  container or not.
* :func:`test_modules_import` actually imports every module; it is skipped
  per-module when a genuinely external runtime dep (pyopensot, xbot2_interface,
  rclpy, ...) is not installed, but still fails on a real breakage inside the
  package.
"""

import ast
import importlib
import pkgutil
from pathlib import Path

import pytest

import tiago_control_node

PKG = tiago_control_node.__name__
PKG_DIR = Path(tiago_control_node.__file__).parent
MODULES = sorted(m.name for m in pkgutil.iter_modules([str(PKG_DIR)]))


def _module_path(modname: str) -> Path:
    return PKG_DIR / f"{modname}.py"


def _toplevel_names(tree: ast.Module) -> set:
    names = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            names.update(t.id for t in node.targets if isinstance(t, ast.Name))
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            names.update(a.asname or a.name.split(".")[0] for a in node.names)
    return names


@pytest.mark.parametrize("modname", MODULES)
def test_source_files_parse(modname):
    """Every module is syntactically valid (catches truncated files)."""
    path = _module_path(modname)
    ast.parse(path.read_text(), filename=str(path))


@pytest.mark.parametrize("modname", MODULES)
def test_intra_package_imports_resolve(modname):
    """`from tiago_control_node[.x] import NAME` only references names that exist."""
    tree = ast.parse(_module_path(modname).read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom) or not node.module:
            continue
        if node.module != PKG and not node.module.startswith(PKG + "."):
            continue

        sub = node.module[len(PKG) + 1:] if node.module != PKG else None
        if sub is not None and sub not in MODULES:
            pytest.fail(f"{modname}: imports from missing submodule {node.module!r}")

        target = _module_path(sub) if sub else PKG_DIR / "__init__.py"
        available = _toplevel_names(ast.parse(target.read_text()))
        for alias in node.names:
            if alias.name == "*" or alias.name in MODULES:
                continue
            assert alias.name in available, (
                f"{modname}: `from {node.module} import {alias.name}` "
                f"but {node.module!r} defines no such name"
            )


@pytest.mark.parametrize("modname", MODULES)
def test_modules_import(modname):
    """Import each module for real; skip only when an external dep is absent."""
    try:
        importlib.import_module(f"{PKG}.{modname}")
    except ModuleNotFoundError as e:
        missing = (e.name or "").split(".")[0]
        if missing == PKG or missing in MODULES:
            raise  # a broken import *inside* the package -- real failure
        pytest.skip(f"external dependency {missing!r} not installed")
