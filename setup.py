import setuptools

with open('requirements.txt', 'r') as f:
    install_requires = f.read().splitlines()

setuptools.setup(
    name='Two-arms-pick-place-planning',
    packages=['Two-arms-pick-place-planning'],
    install_requires=install_requires
    )
