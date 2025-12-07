import React from 'react';
import Link from '@docusaurus/Link';
import styles from './AnimatedCTA.module.css';

interface AnimatedCTAProps {
  text: string;
  link: string;
}

const AnimatedCTA: React.FC<AnimatedCTAProps> = ({ text, link }) => {
  return (
    <Link to={link} className={styles.animatedCTA}>
      <span className={styles.ctaText}>{text}</span>
      <span className={styles.ctaIcon}>→</span>
    </Link>
  );
};

export default AnimatedCTA;
