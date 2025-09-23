/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.vision.config;

/**
 * @param limelightName the name of the limelight to use
 * @param fieldExtentX the extent of the field in the X direction
 * @param fieldExtentY the extent of the field in the Y direction
 * @author Tony Field
 * @since 2025-09-23 16:16
 */
public record LimelightConfig(String limelightName, double fieldExtentX, double fieldExtentY) {}
