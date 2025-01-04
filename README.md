# Runnymede Swerve

## Publishing Maven Package

You need to make a `gradle.properties` in the base directory, containing:

```
gpr.user=<github username>
gpr.token=<private access token>
```

To obtain a private access token, go to [Tokens Classic](https://github.com/settings/tokens) and create a new Token with `write:packages` and `read:packages` permissions.

*Do Not Check This File In*
