from setuptools import setup
from setuptools_rust import RustExtension


setup(name="asm4_solver",
      version="0.0.1",
      rust_extensions=[RustExtension("asm4_solver.solver")],
      packages=["freecad",
                "freecad.asm4_solver",
                "solver"],
      maintainer="Alonso-JAMM",
      url="https://github.com/Alonso-JAMM/Assembly4_Solver",
      description="Solver for Assembly4",
      install_requires=["numpy"],
      include_package_data=True,
      zip_safe=False)
