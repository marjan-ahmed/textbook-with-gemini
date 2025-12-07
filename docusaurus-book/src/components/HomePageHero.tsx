import React from 'react';
import Link from '@docusaurus/Link';
import styles from './HomePageHero.module.css';

interface HomePageHeroProps {
  title: string;
  subtitle: string;
  ctaText: string;
  ctaLink: string;
}

const HomePageHero: React.FC<HomePageHeroProps> = ({ title, subtitle, ctaText, ctaLink }) => {
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>{title}</h1>
          <p className={styles.heroSubtitle}>{subtitle}</p>
          <div className={styles.buttons}>
            <Link className={styles.heroButton} to={ctaLink}>
              {ctaText}
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
};

export default HomePageHero;
