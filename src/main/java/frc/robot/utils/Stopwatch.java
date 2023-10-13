package frc.robot.utils;

/*
 * JBoss, Home of Professional Open Source
 * Copyright 2005, JBoss Inc., and individual contributors as indicated
 * by the @authors tag. See the copyright.txt in the distribution for a
 * full listing of individual contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

import java.io.Serializable;

/**
 * Simulates a stop watch with a <em>lap</em> counter.
 * 
 * @version <tt>$Revision: 2800 $</tt>
 * @author <a href="mailto:jason@planet57.com">Jason Dillon</a>
 */
public class Stopwatch implements Serializable, Cloneable {
  /** The serialVersionUID */
  private static final long serialVersionUID = 4628094303187435707L;

  /** Total time */
  protected long total = 0;

  /** Start time */
  protected long start = -1;

  /** Stop time */
  protected long stop = -1;

  /** The <i>lap</i> count */
  protected int count = 0;

  /** Is the watch started */
  protected boolean running = false;

  /**
   * Default constructor.
   */
  public Stopwatch() {
  }

  /**
   * Construct a StopWatch.
   * 
   * @param running
   *                Start the watch
   */
  public Stopwatch(final boolean running) {
    if (running)
      start();
  }

  /**
   * Start the watch.
   * 
   * @param reset
   *              True to reset the watch prior to starting.
   */
  public void start(final boolean reset) {
    if (!running) {
      if (reset)
        reset();
      start = System.currentTimeMillis();
      running = true;
    }
  }

  /**
   * Start the watch.
   */
  public void start() {
    start(false);
  }

  /**
   * Stop the watch.
   * 
   * @return Elapsed time or 0 if the watch was never started.
   */
  public long stop() {
    long lap = 0;

    if (running) {
      count++;
      stop = System.currentTimeMillis();
      lap = stop - start;
      total += lap;
      running = false;
    }

    return lap;
  }

  /**
   * Reset the watch.
   */
  public void reset() {
    start = -1;
    stop = -1;
    total = 0;
    count = 0;
    running = false;
  }

  /**
   * Get the <i>lap</i> count.
   * 
   * @return The <i>lap</i> count.
   */
  public int getLapCount() {
    return count;
  }

  /**
   * Get the elapsed <i>lap</i> time since the watch was started.
   * 
   * @return Elapsed <i>lap</i> time or 0 if the watch was never started
   */
  public long getLapTime() {
    if (start == -1) {
      return 0;
    } else if (running) {
      return System.currentTimeMillis() - start;
    } else {
      return stop - start;
    }
  }

  /**
   * Get the average <i>lap</i> time since the watch was started.
   * 
   * @return Average <i>lap</i> time since the watch was started.
   */
  public long getAverageLapTime() {
    return (count == 0) ? 0 : getLapTime() / getLapCount();
  }

  /**
   * Get the elapsed time since the watch was created or last reset.
   * 
   * @return Elapsed time or 0 if the watch was never started.
   */
  public long getTime() {
    if (start == -1) {
      return 0;
    } else if (running) {
      return total + System.currentTimeMillis() - start;
    } else {
      return total;
    }
  }

  /**
   * Check if the watch is running.
   * 
   * @return True if the watch is running.
   */
  public boolean isRunning() {
    return running;
  }

  /**
   * Return a string representation.
   */
  public String toString() {
    StringBuffer buff = new StringBuffer();

    if (running) {
      // the watch has not been stopped
      formatElapsedTime(buff, getTime());

      // add the current lap time too if there is more than one lap
      if (count >= 1) {
        buff.append(", count=").append(count);
        buff.append(", current=");
        formatElapsedTime(buff, getLapTime());
      }
    } else {
      // the watch has been stopped
      formatElapsedTime(buff, getTime());

      // add extra info if there is more than one lap
      if (count > 1) {
        buff.append(", count=").append(count);
        buff.append(", average=");
        formatElapsedTime(buff, getAverageLapTime());
      }
    }

    return buff.toString();
  }

  private void formatElapsedTime(final StringBuffer buff, final long lapsed) {
    long m = lapsed / 60000;
    if (m != 0) {
      buff.append(m).append("m:");
    }

    long s = (lapsed - 60000 * m) / 1000;
    if (s != 0) {
      buff.append(s).append("s:");
    }

    // ms is always there, even if it was 0 too
    long ms = (lapsed - 60000 * m - 1000 * s);
    buff.append(ms).append("ms");
  }
}