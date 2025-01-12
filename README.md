# Runnymede Swerve

## Local Development

If you are actively developing this library, or require a local build for some reason, you can build the library
locally.

```bash
./gradlew clean
./gradlew build
./gradlew publishToMavenLocal
```

This will publish the library to your local maven repository, which you can then reference in your project. However,
your project must include the `mavenLocal()` repository. WPILIB projects do not include this by default (they look for a
maven repository in a different location).

_### Code Formatting
This library uses prettier java code formatting. [Be sure to follow the instructions here](https://github.com/jhipster/prettier-java/blob/main/docs/advanced_usage.md)._

## Publishing Maven Package

You need to make a `gradle.properties` in the base directory, containing:

```
gpr.user=<github username>
gpr.token=<private access token>
```

To obtain a private access token, go to [Tokens Classic](https://github.com/settings/tokens) and create a new Token with
`write:packages` and `read:packages` permissions.

*Do Not Check This File In*

## To Publish the First Time

We don't really know exactly what is required here yet but this seems to work

```bash
./gradlew clean publish
./gradlew jreleaserInit
./gradlew jreleaserDeploy
```

## To Publish the Second or Later Time:

```bash
./gradlew clean publish
./gradlew jreleaserDeploy
```
